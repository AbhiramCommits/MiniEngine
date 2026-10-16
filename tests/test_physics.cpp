// Physics tests: narrowphase primitives, resting/bouncing behaviour, raycast and
// sweep queries, and broadphase equivalence.

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "minieng/physics.hpp"
#include "minieng/rng.hpp"

using namespace minieng;

namespace {

constexpr float kDt = 1.0f / 60.0f;

struct TestWorld {
    World world;
    Scene scene;
    PhysicsWorld physics;

    TestWorld() : physics(world, scene) {}

    EntityId add(const Vec3& pos, const Quat& rot, const Collider& collider,
                 const RigidBody& body) {
        const EntityId id = world.create_entity();
        const int node = scene.create_node(-1, id);
        Transform t;
        t.position = pos;
        t.rotation = rot;
        t.scale = Vec3::one();
        scene.set_local(node, t);
        world.add_transform_ref(id, node);
        world.add_collider(id, collider);
        world.add_rigid_body(id, body);
        scene.update_world_transforms();
        return id;
    }

    EntityId add_dynamic_box(const Vec3& pos, const Vec3& half, float restitution = 0.0f,
                             float friction = 0.5f) {
        Collider c;
        c.shape = Collider::AABB;
        c.half_extents = half;
        RigidBody b;
        b.mass = 1.0f;
        b.restitution = restitution;
        b.friction = friction;
        return add(pos, Quat::identity(), c, b);
    }

    EntityId add_static_box(const Vec3& pos, const Vec3& half) {
        Collider c;
        c.shape = Collider::AABB;
        c.half_extents = half;
        RigidBody b;
        b.is_static = true;
        b.mass = 0.0f;
        b.restitution = 0.0f;
        b.friction = 0.5f;
        return add(pos, Quat::identity(), c, b);
    }

    EntityId add_sphere(const Vec3& pos, float radius, bool is_static, float restitution = 0.0f,
                        float friction = 0.4f, const Quat& rot = Quat::identity()) {
        Collider c;
        c.shape = Collider::Sphere;
        c.radius = radius;
        RigidBody b;
        b.is_static = is_static;
        b.mass = is_static ? 0.0f : 1.0f;
        b.restitution = restitution;
        b.friction = friction;
        return add(pos, rot, c, b);
    }

    Vec3 position(EntityId id) const {
        const TransformRef* ref = world.get_transform_ref(id);
        return scene.local_of(ref->scene_node).position;
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Narrowphase primitives
// ---------------------------------------------------------------------------

TEST(Narrowphase, AabbAabbPenetrationAndNormal) {
    Aabb a;
    a.min = Vec3(0, 0, 0);
    a.max = Vec3(2, 2, 2);
    Aabb b;
    b.min = Vec3(1.5f, 0.5f, 0.5f);
    b.max = Vec3(3.5f, 1.5f, 1.5f);

    Vec3 n;
    float pen = 0.0f;
    ASSERT_TRUE(aabb_aabb(a, b, n, pen));
    EXPECT_NEAR(pen, 0.5f, 1e-5f); // overlap only on X
    EXPECT_TRUE(n.approx_equal_to(Vec3::unit_x(), 1e-5f));

    // Fully separated boxes report no overlap.
    Aabb far_box;
    far_box.min = Vec3(10, 10, 10);
    far_box.max = Vec3(11, 11, 11);
    EXPECT_FALSE(aabb_aabb(a, far_box, n, pen));
    EXPECT_FALSE(a.overlaps(far_box));
}

TEST(Narrowphase, SphereSpherePenetrationAndNormal) {
    Vec3 n;
    float pen = 0.0f;
    ASSERT_TRUE(sphere_sphere(Vec3(0, 0, 0), 1.0f, Vec3(1.5f, 0, 0), 1.0f, n, pen));
    EXPECT_NEAR(pen, 0.5f, 1e-5f);
    EXPECT_TRUE(n.approx_equal_to(Vec3::unit_x(), 1e-5f));

    EXPECT_FALSE(sphere_sphere(Vec3(0, 0, 0), 1.0f, Vec3(5, 0, 0), 1.0f, n, pen));

    // Deep overlap uses a fallback axis instead of NaN.
    ASSERT_TRUE(sphere_sphere(Vec3(0, 0, 0), 1.0f, Vec3(0, 0, 0), 1.0f, n, pen));
    EXPECT_NEAR(n.length(), 1.0f, 1e-5f);
}

TEST(Narrowphase, ObbSatDetectsRotatedOverlapMissedByAxisAlignedTest) {
    Obb a;
    a.center = Vec3(0, 0, 0);
    a.half_extents = Vec3(2.0f, 0.1f, 0.1f);

    Obb b;
    b.center = Vec3(0, 1.5f, 0);
    b.half_extents = Vec3(2.0f, 0.1f, 0.1f);
    // Rotate B 90 degrees about Z so it lies along the world Y axis.
    const Quat rot = Quat::from_axis_angle(Vec3::unit_z(), kHalfPi);
    b.axis[0] = rot.rotate_vector(Vec3::unit_x());
    b.axis[1] = rot.rotate_vector(Vec3::unit_y());
    b.axis[2] = rot.rotate_vector(Vec3::unit_z());

    Vec3 n;
    float pen = 0.0f;
    ASSERT_TRUE(obb_obb(a, b, n, pen));
    EXPECT_GT(pen, 0.0f);

    // Treating B as an axis-aligned box at the same center (ignoring rotation)
    // gives no overlap at all.
    Aabb a_axis;
    a_axis.min = a.center - a.half_extents;
    a_axis.max = a.center + a.half_extents;
    Aabb b_axis;
    b_axis.min = b.center - b.half_extents;
    b_axis.max = b.center + b.half_extents;
    EXPECT_FALSE(aabb_aabb(a_axis, b_axis, n, pen));

    // A clearly separated OBB is rejected.
    Obb c = b;
    c.center = Vec3(0, 10.0f, 0);
    EXPECT_FALSE(obb_obb(a, c, n, pen));
}

TEST(Narrowphase, SphereObbPenetration) {
    Obb box;
    box.center = Vec3(0, 0, 0);
    box.half_extents = Vec3(1, 1, 1);

    Vec3 n;
    float pen = 0.0f;
    ASSERT_TRUE(sphere_obb(Vec3(1.5f, 0, 0), 1.0f, box, n, pen));
    EXPECT_NEAR(pen, 0.5f, 1e-5f);
    EXPECT_TRUE(n.approx_equal_to(Vec3::unit_x(), 1e-5f)); // box -> sphere

    EXPECT_FALSE(sphere_obb(Vec3(5, 0, 0), 1.0f, box, n, pen));
}

// ---------------------------------------------------------------------------
// Integration / resolution behaviour
// ---------------------------------------------------------------------------

TEST(Physics, RestingBoxDoesNotSink) {
    TestWorld sim;
    sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(20, 0.5f, 20));
    const EntityId box = sim.add_dynamic_box(Vec3(0, 0.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 0.0f, 0.5f);

    float y200 = 0.0f;
    float previous = 0.0f;
    for (int tick = 0; tick < 300; ++tick) {
        sim.physics.step(kDt);
        const float y = sim.position(box).y;
        EXPECT_GT(y, 0.45f) << "box fell through the floor at tick " << tick;
        if (tick == 199) {
            y200 = y;
        }
        previous = y;
    }

    // Settled and stable: the last tick barely moves and nothing drifted since
    // the settling period.
    EXPECT_NEAR(previous, y200, 1.0e-3f);
    EXPECT_GT(previous, 0.49f);
}

TEST(Physics, BouncingBallApexHeightsDecrease) {
    TestWorld sim;
    sim.physics.config().restitution_slop = 0.2f;
    sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(20, 0.5f, 20));
    const EntityId ball = sim.add_sphere(Vec3(0, 5.0f, 0), 0.5f, false, 0.8f, 0.0f);

    std::vector<float> apexes;
    float prev_y = sim.position(ball).y;
    bool rising = false;
    float peak = prev_y;

    for (int tick = 0; tick < 600; ++tick) {
        sim.physics.step(kDt);
        const float y = sim.position(ball).y;
        if (y > prev_y) {
            rising = true;
            peak = std::max(peak, y);
        } else if (rising) {
            apexes.push_back(peak);
            rising = false;
            peak = y;
        }
        prev_y = y;
    }

    ASSERT_GE(apexes.size(), 3u) << "expected several bounces";
    for (std::size_t i = 1; i < apexes.size(); ++i) {
        EXPECT_LT(apexes[i], apexes[i - 1]) << "apex " << i << " did not decrease";
    }
    // Energy is actually being removed overall.
    EXPECT_LT(apexes.back(), apexes.front());
}

TEST(Physics, StaticBodyUnaffectedByGravity) {
    TestWorld sim;
    const EntityId floor = sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(10, 0.5f, 10));
    for (int tick = 0; tick < 120; ++tick) {
        sim.physics.step(kDt);
    }
    EXPECT_TRUE(sim.position(floor).approx_equal_to(Vec3(0, -0.5f, 0), 1e-6f));
    // And it never accumulates velocity.
    ASSERT_NE(sim.world.get_rigid_body(floor), nullptr);
    EXPECT_FLOAT_EQ(sim.world.get_rigid_body(floor)->velocity.length(), 0.0f);
}

TEST(Physics, HeadOnSpheresExchangeMomentum) {
    TestWorld sim;
    const EntityId a = sim.add_sphere(Vec3(0.9f, 0, 0), 1.0f, false, 1.0f, 0.0f);
    const EntityId b = sim.add_sphere(Vec3(1.1f, 0, 0), 1.0f, false, 1.0f, 0.0f);
    sim.world.get_rigid_body(a)->velocity = Vec3(1.0f, 0, 0);
    sim.world.get_rigid_body(b)->velocity = Vec3(-1.0f, 0, 0);

    Contact contact;
    contact.a = a;
    contact.b = b;
    contact.normal = Vec3::unit_x();
    contact.penetration = 0.1f;
    contact.point = Vec3(1.0f, 0, 0);

    sim.physics.resolve_contacts({contact}, kDt);

    // Equal masses, elastic: velocities swap.
    EXPECT_NEAR(sim.world.get_rigid_body(a)->velocity.x, -1.0f, 1e-3f);
    EXPECT_NEAR(sim.world.get_rigid_body(b)->velocity.x, 1.0f, 1e-3f);
}

TEST(Physics, FrictionSlowsSlidingBox) {
    TestWorld sim;
    sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(20, 0.5f, 20));
    const EntityId box = sim.add_dynamic_box(Vec3(0, 0.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 0.0f, 0.8f);
    sim.world.get_rigid_body(box)->velocity = Vec3(5.0f, 0, 0);

    for (int tick = 0; tick < 60; ++tick) {
        sim.physics.step(kDt);
    }
    const float speed = std::fabs(sim.world.get_rigid_body(box)->velocity.x);
    EXPECT_LT(speed, 5.0f);
    EXPECT_GE(speed, 0.0f);
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

TEST(Query, RaycastHitsSphereAtAnalyticDistance) {
    TestWorld sim;
    sim.add_sphere(Vec3(0, 0, 0), 2.0f, true);
    sim.physics.build_broadphase();

    const RayHit hit = sim.physics.raycast(Vec3(-10, 0, 0), Vec3(1, 0, 0), 100.0f);
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(hit.t, 8.0f, 1e-4f);
    EXPECT_TRUE(hit.point.approx_equal_to(Vec3(-2, 0, 0), 1e-3f));
    EXPECT_TRUE(hit.normal.approx_equal_to(Vec3(-1, 0, 0), 1e-3f));
}

TEST(Query, RaycastHitsBoxAtAnalyticDistance) {
    TestWorld sim;
    sim.add_static_box(Vec3(0, 0, 0), Vec3(1, 1, 1));
    sim.physics.build_broadphase();

    const RayHit hit = sim.physics.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0), 100.0f);
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(hit.t, 4.0f, 1e-4f);
    EXPECT_TRUE(hit.point.approx_equal_to(Vec3(-1, 0, 0), 1e-3f));
    EXPECT_TRUE(hit.normal.approx_equal_to(Vec3(-1, 0, 0), 1e-3f));

    // An oblique ray hits the top face at the analytic distance.
    const RayHit top = sim.physics.raycast(Vec3(0, 5, 0), Vec3(0, -1, 0), 100.0f);
    ASSERT_TRUE(top.hit);
    EXPECT_NEAR(top.t, 4.0f, 1e-4f);
    EXPECT_TRUE(top.normal.approx_equal_to(Vec3(0, 1, 0), 1e-3f));
}

TEST(Query, RaycastMissReturnsNoHit) {
    TestWorld sim;
    sim.add_static_box(Vec3(0, 0, 0), Vec3(1, 1, 1));
    sim.add_sphere(Vec3(0, 0, 0), 1.0f, true);
    sim.physics.build_broadphase();

    const RayHit miss = sim.physics.raycast(Vec3(-5, 10, 0), Vec3(1, 0, 0), 100.0f);
    EXPECT_FALSE(miss.hit);

    // Too short a max distance also misses.
    EXPECT_FALSE(sim.physics.raycast(Vec3(-5, 0, 0), Vec3(1, 0, 0), 2.0f).hit);
    // Zero-length direction is rejected.
    EXPECT_FALSE(sim.physics.raycast(Vec3(-5, 0, 0), Vec3(0, 0, 0), 100.0f).hit);
}

TEST(Query, RaycastReturnsNearestEntity) {
    TestWorld sim;
    const EntityId near_sphere = sim.add_sphere(Vec3(0, 0, 0), 1.0f, true);
    const EntityId far_sphere = sim.add_sphere(Vec3(5, 0, 0), 1.0f, true);
    sim.physics.build_broadphase();

    const RayHit hit = sim.physics.raycast(Vec3(-10, 0, 0), Vec3(1, 0, 0), 100.0f);
    ASSERT_TRUE(hit.hit);
    EXPECT_EQ(hit.entity, near_sphere);
    EXPECT_NE(hit.entity, far_sphere);
    EXPECT_NEAR(hit.t, 9.0f, 1e-4f);
}

TEST(Query, SweepDetectsTunnelingCaseDiscreteTestMisses) {
    TestWorld sim;
    sim.add_static_box(Vec3(5, 0, 0), Vec3(0.05f, 5.0f, 5.0f));
    sim.physics.build_broadphase();

    // A 1x1x1 box teleporting from x=-5 to x=+10 passes straight through a thin
    // wall. Endpoint overlap tests see nothing.
    Aabb source;
    source.min = Vec3(-5.5f, -0.5f, -0.5f);
    source.max = Vec3(-4.5f, 0.5f, 0.5f);
    Aabb at_destination;
    at_destination.min = Vec3(9.5f, -0.5f, -0.5f);
    at_destination.max = Vec3(10.5f, 0.5f, 0.5f);
    const Aabb wall = sim.physics.world_aabb(sim.world.entities().front());
    EXPECT_FALSE(source.overlaps(wall));
    EXPECT_FALSE(at_destination.overlaps(wall));

    const SweepHit hit = sim.physics.sweep_aabb(source, Vec3(-5, 0, 0), Vec3(10, 0, 0));
    ASSERT_TRUE(hit.hit) << "sweep should not tunnel through the wall";
    // Contact when the moving box's leading face reaches the wall:
    // wall min.x = 5 - 0.05 = 4.95, expanded by the 0.5 half extent -> 4.45.
    EXPECT_NEAR(hit.t, (4.45f + 5.0f) / 15.0f, 1e-3f);
    EXPECT_TRUE(hit.normal.approx_equal_to(Vec3(-1, 0, 0), 1e-3f));
}

TEST(Query, SweepReportsNoHitWhenPathIsClear) {
    TestWorld sim;
    sim.add_static_box(Vec3(5, 0, 0), Vec3(0.05f, 1.0f, 1.0f));
    sim.physics.build_broadphase();

    Aabb box;
    box.min = Vec3(-0.5f, -0.5f, -0.5f);
    box.max = Vec3(0.5f, 0.5f, 0.5f);

    // Pass well above the wall.
    EXPECT_FALSE(sim.physics.sweep_aabb(box, Vec3(-5, 10, 0), Vec3(10, 10, 0)).hit);

    // And a zero-length motion produces nothing.
    EXPECT_FALSE(sim.physics.sweep_aabb(box, Vec3(-5, 0, 0), Vec3(-5, 0, 0)).hit);
}

TEST(Query, OverlapSphereReturnsContainingEntities) {
    TestWorld sim;
    const EntityId inside = sim.add_sphere(Vec3(0, 0, 0), 1.0f, true);
    const EntityId near_edge = sim.add_sphere(Vec3(2.5f, 0, 0), 1.0f, true);
    const EntityId far_away = sim.add_sphere(Vec3(20, 0, 0), 1.0f, true);
    sim.physics.build_broadphase();

    const std::vector<EntityId> hits = sim.physics.overlap_sphere(Vec3(0, 0, 0), 2.0f);
    EXPECT_NE(std::find(hits.begin(), hits.end(), inside), hits.end());
    EXPECT_NE(std::find(hits.begin(), hits.end(), near_edge), hits.end());
    EXPECT_EQ(std::find(hits.begin(), hits.end(), far_away), hits.end());
    EXPECT_TRUE(std::is_sorted(hits.begin(), hits.end()));
}

// ---------------------------------------------------------------------------
// Broadphase
// ---------------------------------------------------------------------------

TEST(Broadphase, SweepAndPruneEqualsBruteForceOnRandomScenes) {
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        Rng rng(seed);
        const int count = rng.range_int(1, 40);
        std::vector<BroadphaseBody> bodies;
        bodies.reserve(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            BroadphaseBody body;
            body.id = static_cast<EntityId>(i + 1);
            const Vec3 c(rng.range(-10.0f, 10.0f), rng.range(-10.0f, 10.0f),
                         rng.range(-10.0f, 10.0f));
            const Vec3 e(rng.range(0.1f, 2.0f), rng.range(0.1f, 2.0f), rng.range(0.1f, 2.0f));
            body.aabb.min = c - e;
            body.aabb.max = c + e;
            bodies.push_back(body);
        }

        const std::vector<Pair> sap = broadphase_sweep_and_prune(bodies);
        const std::vector<Pair> brute = broadphase_brute_force(bodies);

        ASSERT_EQ(sap.size(), brute.size()) << "seed " << seed;
        for (std::size_t i = 0; i < sap.size(); ++i) {
            EXPECT_EQ(sap[i].a, brute[i].a) << "seed " << seed;
            EXPECT_EQ(sap[i].b, brute[i].b) << "seed " << seed;
        }
    }
}

TEST(Broadphase, DetectsOverlappingPairAndSkipsSeparated) {
    std::vector<BroadphaseBody> bodies(3);
    bodies[0].id = 10;
    bodies[0].aabb.min = Vec3(0, 0, 0);
    bodies[0].aabb.max = Vec3(1, 1, 1);
    bodies[1].id = 20;
    bodies[1].aabb.min = Vec3(0.5f, 0.5f, 0.5f);
    bodies[1].aabb.max = Vec3(1.5f, 1.5f, 1.5f);
    bodies[2].id = 30;
    bodies[2].aabb.min = Vec3(50, 0, 0);
    bodies[2].aabb.max = Vec3(51, 1, 1);

    const std::vector<Pair> sap = broadphase_sweep_and_prune(bodies);
    ASSERT_EQ(sap.size(), 1u);
    EXPECT_EQ(sap[0].a, 10u);
    EXPECT_EQ(sap[0].b, 20u);
}

// ---------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------

TEST(Physics, TwoIndependentRunsProduceIdenticalHashes) {
    const auto build_and_run = [](std::uint64_t seed, int ticks) -> std::uint64_t {
        TestWorld sim;
        sim.physics.config().restitution_slop = 0.2f;
        sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(15, 0.5f, 15));

        Rng rng(seed);
        for (int i = 0; i < 30; ++i) {
            const Vec3 pos(rng.range(-5.0f, 5.0f), rng.range(1.0f, 8.0f), rng.range(-5.0f, 5.0f));
            Collider c;
            c.shape = (i % 2 == 0) ? Collider::AABB : Collider::Sphere;
            c.half_extents = Vec3(0.4f, 0.4f, 0.4f);
            c.radius = 0.4f;
            RigidBody b;
            b.mass = rng.range(0.5f, 3.0f);
            b.restitution = 0.3f;
            b.friction = 0.4f;
            const Quat rot = Quat::from_axis_angle(
                Vec3(rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f)),
                rng.range(0.0f, kPi));
            sim.add(pos, rot, c, b);
        }

        for (int tick = 0; tick < ticks; ++tick) {
            sim.physics.step(kDt);
        }
        return sim.world.hash(sim.scene);
    };

    const std::uint64_t a = build_and_run(0xBEEF01u, 240);
    const std::uint64_t b = build_and_run(0xBEEF01u, 240);
    const std::uint64_t c = build_and_run(0xBEEF02u, 240);

    EXPECT_EQ(a, b) << "identical seeded runs must match";
    EXPECT_NE(a, c) << "different seeds must diverge";
}

TEST(Physics, SapAndBruteForceProduceSameContacts) {
    TestWorld sim;
    sim.add_static_box(Vec3(0, -0.5f, 0), Vec3(20, 0.5f, 20));
    for (int i = 0; i < 12; ++i) {
        sim.add_dynamic_box(Vec3(0.3f * static_cast<float>(i) - 1.5f, 0.5f + 0.05f * i, 0.0f),
                            Vec3(0.4f, 0.4f, 0.4f));
    }
    sim.physics.build_broadphase();

    // Both generators must see the exact same set of bodies, so compare them on
    // one shared broadphase snapshot (running resolve first would move bodies).
    const std::vector<BroadphaseBody>& bodies = sim.physics.broadphase_bodies();
    ASSERT_FALSE(bodies.empty());

    const std::vector<Pair> sap_pairs = broadphase_sweep_and_prune(bodies);
    const std::vector<Pair> brute_pairs = broadphase_brute_force(bodies);

    ASSERT_EQ(sap_pairs.size(), brute_pairs.size());
    EXPECT_GT(sap_pairs.size(), 0u);
    for (std::size_t i = 0; i < sap_pairs.size(); ++i) {
        EXPECT_EQ(sap_pairs[i].a, brute_pairs[i].a);
        EXPECT_EQ(sap_pairs[i].b, brute_pairs[i].b);
    }

    const std::vector<Contact> sap_contacts = sim.physics.narrowphase(sap_pairs);
    const std::vector<Contact> brute_contacts = sim.physics.narrowphase(brute_pairs);
    ASSERT_EQ(sap_contacts.size(), brute_contacts.size());
    for (std::size_t i = 0; i < sap_contacts.size(); ++i) {
        EXPECT_EQ(sap_contacts[i].a, brute_contacts[i].a);
        EXPECT_EQ(sap_contacts[i].b, brute_contacts[i].b);
        EXPECT_NEAR(sap_contacts[i].penetration, brute_contacts[i].penetration, 1e-6f);
    }

    // The SAP flag on the world routes through the same generators.
    sim.physics.set_use_sap(true);
    EXPECT_EQ(sim.physics.broadphase_pairs().size(), brute_pairs.size());
    sim.physics.set_use_sap(false);
    EXPECT_EQ(sim.physics.broadphase_pairs().size(), brute_pairs.size());
}
