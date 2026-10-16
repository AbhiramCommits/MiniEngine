// Deterministic replay: record 1200 ticks of input to data/replays/demo.replay,
// then replay the file and require identical per-tick world hashes.

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

#include "minieng/loop.hpp"
#include "minieng/physics.hpp"
#include "minieng/replay.hpp"
#include "minieng/rng.hpp"
#include "minieng/scene.hpp"
#include "minieng/world.hpp"

using namespace minieng;

namespace {

constexpr float kDt = 1.0f / 60.0f;
constexpr int kReplayTicks = 1200;

std::string replay_path() {
    return std::string(MINIENGINE_SOURCE_DIR) + "/data/replays/demo.replay";
}

// A small sandbox driven only by its recorded input.
struct ReplaySim {
    World world;
    Scene scene;
    PhysicsWorld physics{world, scene};
    FixedLoop loop{kDt, 8};

    EntityId player = kInvalidEntity;
    float input_x = 0.0f;
    float input_y = 0.0f;

    void build(std::uint64_t seed) {
        physics.config().gravity = Vec3(0.0f, -9.81f, 0.0f);
        physics.config().restitution_slop = 0.2f;

        // Static floor.
        add_box(Vec3(0.0f, -0.5f, 0.0f), Vec3(20.0f, 0.5f, 20.0f), true, 0.0f);

        Rng rng(seed);
        for (int i = 0; i < 24; ++i) {
            add_sphere(Vec3(rng.range(-6.0f, 6.0f), rng.range(1.0f, 9.0f), rng.range(-6.0f, 6.0f)),
                       0.4f, 0.5f);
        }

        player = add_sphere(Vec3(0.0f, 1.5f, 0.0f), 0.6f, 0.4f);
        world.get_rigid_body(player)->mass = 2.0f;

        loop.set_system(SystemStage::Input, [this](float dt) {
            RigidBody* body = world.get_rigid_body(player);
            if (body != nullptr) {
                body->velocity.x += input_x * 12.0f * dt;
                body->velocity.z += input_y * 12.0f * dt;
            }
        });
        loop.set_system(SystemStage::PhysicsIntegrate, [this](float dt) { physics.integrate(dt); });
        loop.set_system(SystemStage::CollisionResolve, [this](float dt) {
            physics.build_broadphase();
            const std::vector<Pair> pairs = physics.broadphase_pairs();
            const std::vector<Contact> contacts = physics.narrowphase(pairs);
            physics.resolve_contacts(contacts, dt);
        });
        loop.set_system(SystemStage::SceneTransform,
                        [this](float) { scene.update_world_transforms(); });

        scene.update_world_transforms();
    }

    EntityId add_box(const Vec3& position, const Vec3& half, bool is_static, float restitution) {
        const EntityId id = world.create_entity();
        const int node = scene.create_node(-1, id);
        Transform t;
        t.position = position;
        scene.set_local(node, t);
        world.add_transform_ref(id, node);
        Collider c;
        c.shape = Collider::AABB;
        c.half_extents = half;
        world.add_collider(id, c);
        RigidBody b;
        b.is_static = is_static;
        b.mass = is_static ? 0.0f : 1.0f;
        b.restitution = restitution;
        b.friction = 0.5f;
        world.add_rigid_body(id, b);
        return id;
    }

    EntityId add_sphere(const Vec3& position, float radius, float restitution) {
        const EntityId id = world.create_entity();
        const int node = scene.create_node(-1, id);
        Transform t;
        t.position = position;
        scene.set_local(node, t);
        world.add_transform_ref(id, node);
        Collider c;
        c.shape = Collider::Sphere;
        c.radius = radius;
        world.add_collider(id, c);
        RigidBody b;
        b.mass = 1.0f;
        b.restitution = restitution;
        b.friction = 0.4f;
        world.add_rigid_body(id, b);
        return id;
    }

    std::uint64_t step_once() {
        loop.step_once();
        return world.hash(scene);
    }
};

// Generates the input script and records the resulting hashes.
Replay record_replay(std::uint64_t seed) {
    ReplaySim sim;
    sim.build(seed);

    Rng rng(seed ^ 0xA5A5A5A5ull);
    Replay replay;
    replay.seed = seed;
    replay.frames.reserve(kReplayTicks);

    for (int tick = 0; tick < kReplayTicks; ++tick) {
        sim.input_x = rng.range(-1.0f, 1.0f);
        sim.input_y = rng.range(-1.0f, 1.0f);
        const std::uint64_t hash = sim.step_once();
        ReplayFrame frame;
        frame.input_x = sim.input_x;
        frame.input_y = sim.input_y;
        frame.hash = hash;
        replay.frames.push_back(frame);
    }
    return replay;
}

// Re-runs a replay and returns the per-tick hashes it produces.
std::vector<std::uint64_t> replay_hashes(const Replay& replay) {
    ReplaySim sim;
    sim.build(replay.seed);

    std::vector<std::uint64_t> hashes;
    hashes.reserve(replay.frames.size());
    for (const ReplayFrame& frame : replay.frames) {
        sim.input_x = frame.input_x;
        sim.input_y = frame.input_y;
        hashes.push_back(sim.step_once());
    }
    return hashes;
}

} // namespace

TEST(Determinism, ReplayFileRecordsTwelveHundredTicks) {
    const std::string path = replay_path();
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());

    const Replay recorded = record_replay(20261016u);
    ASSERT_EQ(recorded.frames.size(), static_cast<std::size_t>(kReplayTicks));
    ASSERT_TRUE(recorded.write_to_file(path)) << path;

    // Round-trip through the file.
    const Replay loaded = Replay::read_from_file(path);
    ASSERT_EQ(loaded.frames.size(), static_cast<std::size_t>(kReplayTicks));
    EXPECT_EQ(loaded.seed, recorded.seed);
    EXPECT_TRUE(loaded.same_as(recorded));
}

TEST(Determinism, ReplayingTheFileReproducesEveryTickHash) {
    const std::string path = replay_path();
    const Replay loaded = Replay::read_from_file(path);
    ASSERT_EQ(loaded.frames.size(), static_cast<std::size_t>(kReplayTicks))
        << "run the recording test first (or together) to generate " << path;

    const std::vector<std::uint64_t> replayed = replay_hashes(loaded);
    ASSERT_EQ(replayed.size(), loaded.frames.size());

    for (std::size_t tick = 0; tick < replayed.size(); ++tick) {
        EXPECT_EQ(replayed[tick], loaded.frames[tick].hash) << "tick " << tick;
    }

    // The simulation is actually evolving, so the hashes are not all identical.
    EXPECT_NE(replayed.front(), replayed.back());
}

TEST(Determinism, TwoIndependentSimulationsProduceTheSameTicks) {
    const Replay recorded = record_replay(4242u);
    const std::vector<std::uint64_t> again = replay_hashes(recorded);
    ASSERT_EQ(again.size(), recorded.frames.size());
    for (std::size_t tick = 0; tick < again.size(); ++tick) {
        EXPECT_EQ(again[tick], recorded.frames[tick].hash) << "tick " << tick;
    }
}

TEST(Determinism, ChangingOneInputDivergesAndIsDetected) {
    Replay replay = record_replay(777u);
    ASSERT_FALSE(replay.frames.empty());

    // Perturb one mid-script input and confirm the replay no longer matches.
    Replay perturbed = replay;
    perturbed.frames[100].input_x += 0.5f;

    const std::vector<std::uint64_t> hashes = replay_hashes(perturbed);
    std::size_t mismatches = 0;
    for (std::size_t tick = 0; tick < hashes.size(); ++tick) {
        if (hashes[tick] != perturbed.frames[tick].hash) {
            ++mismatches;
        }
    }
    EXPECT_GT(mismatches, 0u) << "a changed input must be observable in the hashes";
    EXPECT_FALSE(perturbed.same_as(replay));
}

TEST(Determinism, DifferentSeedsDivergeEarly) {
    const Replay a = record_replay(11u);
    const Replay b = record_replay(12u);
    ASSERT_EQ(a.frames.size(), b.frames.size());
    bool differs = false;
    for (std::size_t i = 0; i < a.frames.size(); ++i) {
        if (a.frames[i].hash != b.frames[i].hash) {
            differs = true;
            break;
        }
    }
    EXPECT_TRUE(differs);
}
