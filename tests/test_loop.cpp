// Fixed-loop tests: accumulator / substep behaviour, explicit system ordering,
// and cross-instance determinism of a seeded scene.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "minieng/loop.hpp"
#include "minieng/rng.hpp"
#include "minieng/scene.hpp"
#include "minieng/world.hpp"

using namespace minieng;

namespace {

constexpr float kDt = 1.0f / 60.0f;

// Builds an independent world+scene pair with the same seeded initial state.
struct Sim {
    World world;
    Scene scene;
    std::uint64_t seed = 1;

    std::uint64_t run(int ticks, std::vector<std::uint64_t>* per_tick = nullptr) {
        FixedLoop loop(kDt, 8);
        const Vec3 gravity(0.0f, -9.81f, 0.0f);

        loop.set_system(SystemStage::PhysicsIntegrate, [this, gravity](float dt) {
            for (const EntityId id : world.entities()) {
                RigidBody* body = world.get_rigid_body(id);
                if (body == nullptr || body->is_static) {
                    continue;
                }
                body->velocity = body->velocity + gravity * dt;
                TransformRef* ref = world.get_transform_ref(id);
                if (ref == nullptr || ref->scene_node < 0) {
                    continue;
                }
                Transform& t = scene.local_of(ref->scene_node);
                t.position = t.position + body->velocity * dt;
            }
        });
        loop.set_system(SystemStage::SceneTransform,
                        [this](float) { scene.update_world_transforms(); });

        for (int i = 0; i < ticks; ++i) {
            loop.step_once();
            if (per_tick != nullptr) {
                per_tick->push_back(world.hash(scene));
            }
        }
        return world.hash(scene);
    }
};

Sim make_sim(std::uint64_t seed, int bodies) {
    Sim sim;
    sim.seed = seed;

    Rng rng(seed);
    for (int i = 0; i < bodies; ++i) {
        const EntityId id = sim.world.create_entity();
        const int node = sim.scene.create_node(-1, id);
        sim.world.add_transform_ref(id, node);

        Transform t;
        t.position = Vec3(rng.range(-5.0f, 5.0f), rng.range(1.0f, 10.0f), rng.range(-5.0f, 5.0f));
        t.rotation = Quat::from_axis_angle(Vec3(rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f),
                                                rng.range(-1.0f, 1.0f)),
                                           rng.range(0.0f, kTwoPi));
        sim.scene.set_local(node, t);

        RigidBody body;
        body.velocity = Vec3(rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f));
        body.mass = rng.range(0.5f, 4.0f);
        sim.world.add_rigid_body(id, body);
    }
    sim.scene.update_world_transforms();
    return sim;
}

}  // namespace

TEST(FixedLoop, DefaultsAndAccessors) {
    FixedLoop loop;
    EXPECT_NEAR(loop.dt(), kDt, 1e-9f);
    EXPECT_EQ(loop.tick_count(), 0u);
    EXPECT_FLOAT_EQ(loop.accumulator(), 0.0f);
    EXPECT_FLOAT_EQ(loop.alpha(), 0.0f);

    // Invalid dt / substep values are rejected rather than accepted silently.
    loop.set_dt(-1.0f);
    EXPECT_NEAR(loop.dt(), kDt, 1e-9f);
    loop.set_max_substeps(0);
    EXPECT_EQ(loop.max_substeps(), 8);
}

TEST(FixedLoop, AccumulatorProducesExpectedSubsteps) {
    FixedLoop loop(kDt, 8);
    int ticks = 0;
    loop.set_system(SystemStage::Input, [&ticks](float) { ++ticks; });

    // 1.5 steps worth of time -> 1 step, half a step left over.
    EXPECT_EQ(loop.advance(kDt * 1.5f), 1);
    EXPECT_EQ(loop.tick_count(), 1u);
    EXPECT_NEAR(loop.accumulator(), kDt * 0.5f, 1e-6f);
    EXPECT_NEAR(loop.alpha(), 0.5f, 1e-4f);

    // The remaining half step plus another half yields a second step.
    EXPECT_EQ(loop.advance(kDt * 0.5f), 1);
    EXPECT_EQ(loop.tick_count(), 2u);
    EXPECT_NEAR(loop.accumulator(), 0.0f, 1e-6f);

    // Non-positive frame deltas do no work.
    EXPECT_EQ(loop.advance(0.0f), 0);
    EXPECT_EQ(loop.advance(-5.0f), 0);
    EXPECT_EQ(ticks, 2);

    loop.reset();
    EXPECT_EQ(loop.tick_count(), 0u);
    EXPECT_FLOAT_EQ(loop.accumulator(), 0.0f);
}

TEST(FixedLoop, MaxSubstepsCapsCatchUp) {
    FixedLoop loop(kDt, 4);
    int ticks = 0;
    loop.set_system(SystemStage::Input, [&ticks](float) { ++ticks; });

    // A full second of lag wants 60 steps; the cap allows only 4 and the
    // backlog is clamped so the loop cannot spiral.
    EXPECT_EQ(loop.advance(1.0f), 4);
    EXPECT_EQ(ticks, 4);
    EXPECT_LE(loop.accumulator(), kDt + 1e-6f);
}

TEST(FixedLoop, SystemsRunInStageOrder) {
    FixedLoop loop(kDt, 8);
    std::vector<std::string> order;

    loop.set_system(SystemStage::UiObserve, [&order](float) { order.push_back("ui"); });
    loop.set_system(SystemStage::Input, [&order](float) { order.push_back("input"); });
    loop.set_system(SystemStage::CollisionResolve, [&order](float) { order.push_back("resolve"); });
    loop.set_system(SystemStage::SceneTransform, [&order](float) { order.push_back("scene"); });
    loop.set_system(SystemStage::PhysicsIntegrate, [&order](float) { order.push_back("integrate"); });
    loop.set_system(SystemStage::Spawn, [&order](float) { order.push_back("spawn"); });

    loop.step_once();

    const std::vector<std::string> expected = {"input", "spawn", "integrate",
                                               "resolve", "scene", "ui"};
    EXPECT_EQ(order, expected);

    // Systems receive the fixed dt, not the frame dt.
    float seen_dt = -1.0f;
    loop.set_system(SystemStage::Input, [&seen_dt](float dt) { seen_dt = dt; });
    loop.advance(0.5f);
    EXPECT_NEAR(seen_dt, kDt, 1e-9f);

    EXPECT_STREQ(system_stage_name(SystemStage::CollisionResolve), "collision_resolve");
}

TEST(Determinism, SameSeedDifferentInstancesMatchFor600Ticks) {
    Sim a = make_sim(0xC0FFEEu, 40);
    Sim b = make_sim(0xC0FFEEu, 40);

    std::vector<std::uint64_t> hashes_a;
    std::vector<std::uint64_t> hashes_b;
    const std::uint64_t final_a = a.run(600, &hashes_a);
    const std::uint64_t final_b = b.run(600, &hashes_b);

    ASSERT_EQ(hashes_a.size(), 600u);
    ASSERT_EQ(hashes_b.size(), 600u);
    for (std::size_t i = 0; i < hashes_a.size(); ++i) {
        EXPECT_EQ(hashes_a[i], hashes_b[i]) << "tick " << i;
    }
    EXPECT_EQ(final_a, final_b);

    // The simulation must actually be changing (a frozen hash would be a bug).
    EXPECT_NE(hashes_a.front(), hashes_a.back());
}

TEST(Determinism, DifferentSeedProducesDifferentHash) {
    Sim a = make_sim(1234u, 40);
    Sim b = make_sim(4321u, 40);
    EXPECT_NE(a.run(200), b.run(200));
}

TEST(Determinism, RngIsReproducible) {
    Rng a(999u);
    Rng b(999u);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_EQ(a.next_u64(), b.next_u64());
    }

    Rng c(1000u);
    // Different seeds diverge immediately.
    EXPECT_NE(Rng(1u).next_u64(), c.next_u64());

    // next_float stays in range.
    Rng r(7u);
    for (int i = 0; i < 100; ++i) {
        const float f = r.next_float();
        EXPECT_GE(f, 0.0f);
        EXPECT_LT(f, 1.0f);
    }
}

TEST(World, ComponentStorageAndDeterministicIteration) {
    World world;
    Scene scene;

    const EntityId a = world.create_entity();
    const EntityId b = world.create_entity();
    const EntityId c = world.create_entity();

    world.add_transform_ref(a, scene.create_node(-1, a));
    world.add_transform_ref(b, scene.create_node(-1, b));
    world.add_rigid_body(c, RigidBody{});
    world.add_tag(a, "floor");

    EXPECT_TRUE(world.alive(a));
    EXPECT_EQ(world.entity_count(), 3u);
    EXPECT_TRUE(world.has_transform_ref(a));
    EXPECT_FALSE(world.has_rigid_body(a));
    EXPECT_TRUE(world.has_rigid_body(c));
    ASSERT_NE(world.get_tag(a), nullptr);
    EXPECT_EQ(world.get_tag(a)->name, "floor");

    // Ascending id order, always.
    const std::vector<EntityId>& ids = world.entities();
    EXPECT_TRUE(std::is_sorted(ids.begin(), ids.end()));

    world.destroy_entity(a);
    EXPECT_FALSE(world.alive(a));
    EXPECT_EQ(world.entity_count(), 2u);
    EXPECT_EQ(world.index_of(b), 0);
    EXPECT_EQ(world.index_of(c), 1);
    EXPECT_TRUE(std::is_sorted(world.entities().begin(), world.entities().end()));

    const auto bodies = world.entities_with_rigid_body();
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_EQ(bodies[0], c);
}
