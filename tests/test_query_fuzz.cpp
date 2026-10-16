// Query fuzzing: 10k seeded random rays and 10k seeded random AABB sweeps are
// compared against brute-force reference implementations that test every entity
// directly (no broadphase acceleration). Both must agree on the nearest hit
// entity and on the hit parameter.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "minieng/physics.hpp"
#include "minieng/rng.hpp"
#include "minieng/scene.hpp"
#include "minieng/world.hpp"

using namespace minieng;

namespace {

struct FuzzScene {
    World world;
    Scene scene;
    PhysicsWorld physics{world, scene};

    void build(std::uint64_t seed) {
        Rng rng(seed);

        // Static floor keeps at least one large, predictable obstacle around.
        add(Vec3(0.0f, -0.5f, 0.0f), Quat::identity(), Collider::AABB, Vec3(12.0f, 0.5f, 12.0f),
            1.0f, true);

        for (int i = 0; i < 150; ++i) {
            const Vec3 position(rng.range(-9.0f, 9.0f), rng.range(0.2f, 9.0f),
                                rng.range(-9.0f, 9.0f));
            const int kind = rng.range_int(0, 2);
            const Quat rotation = Quat::from_axis_angle(
                Vec3(rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f)),
                rng.range(0.0f, kPi));
            const Collider::Shape shape = (kind == 0) ? Collider::AABB
                                        : (kind == 1) ? Collider::Sphere
                                                      : Collider::OBB;
            add(position, rotation, shape,
                Vec3(rng.range(0.2f, 1.0f), rng.range(0.2f, 1.0f), rng.range(0.2f, 1.0f)),
                rng.range(0.2f, 0.8f), false);
        }

        scene.update_world_transforms();
        physics.build_broadphase();
    }

    void add(const Vec3& position, const Quat& rotation, Collider::Shape shape, const Vec3& extents,
             float radius, bool is_static) {
        const EntityId id = world.create_entity();
        const int node = scene.create_node(-1, id);
        Transform t;
        t.position = position;
        t.rotation = rotation;
        scene.set_local(node, t);
        world.add_transform_ref(id, node);

        Collider c;
        c.shape = shape;
        c.half_extents = extents;
        c.radius = radius;
        world.add_collider(id, c);

        RigidBody b;
        b.is_static = is_static;
        b.mass = is_static ? 0.0f : 1.0f;
        world.add_rigid_body(id, b);
    }

    // Brute force: every entity, no acceleration, ascending id order.
    // The direction is normalized here exactly once, matching what
    // PhysicsWorld::raycast does internally.
    RayHit reference_raycast(const Vec3& origin, const Vec3& dir, float max_t) const {
        const Vec3 d = dir.normalized();
        RayHit best;
        best.t = max_t;
        for (const EntityId id : world.entities()) {
            if (world.get_collider(id) == nullptr) {
                continue;
            }
            const RayHit hit = physics.raycast_entity(id, origin, d, best.t);
            if (hit.hit && hit.t < best.t) {
                best = hit;
            }
        }
        return best;
    }

    SweepHit reference_sweep(const Vec3& half, const Vec3& from, const Vec3& to) const {
        SweepHit best;
        best.t = 1.0f;
        for (const EntityId id : world.entities()) {
            if (world.get_collider(id) == nullptr) {
                continue;
            }
            const Aabb obstacle = physics.world_aabb(id);
            const SweepHit hit = sweep_aabb_vs_aabb(half, from, to, obstacle, id);
            if (hit.hit && hit.t < best.t) {
                best = hit;
            }
        }
        return best;
    }
};

} // namespace

TEST(QueryFuzz, TenThousandRaysMatchBruteForce) {
    FuzzScene fuzz;
    fuzz.build(0x5EED01u);

    Rng rng(0x1234u);
    std::size_t hits = 0;
    std::size_t misses = 0;

    const int kRays = 10000;
    for (int i = 0; i < kRays; ++i) {
        const Vec3 origin(rng.range(-15.0f, 15.0f), rng.range(-2.0f, 15.0f),
                          rng.range(-15.0f, 15.0f));
        // Deliberately not pre-normalized: raycast() and the reference both
        // normalize exactly once, so they see bit-identical directions.
        const Vec3 dir(rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f), rng.range(-1.0f, 1.0f));
        const float max_t = 60.0f;

        const RayHit accelerated = fuzz.physics.raycast(origin, dir, max_t);
        const RayHit reference = fuzz.reference_raycast(origin, dir, max_t);

        ASSERT_EQ(accelerated.hit, reference.hit) << "ray " << i;
        if (reference.hit) {
            ++hits;
            EXPECT_NEAR(accelerated.t, reference.t, 1.0e-4f) << "ray " << i;
            EXPECT_EQ(accelerated.entity, reference.entity) << "ray " << i;
        } else {
            ++misses;
        }
    }

    // Sanity: the fuzz has to actually exercise both outcomes.
    EXPECT_GT(hits, 0u);
    EXPECT_GT(misses, 0u);
}

TEST(QueryFuzz, TenThousandSweepsMatchBruteForce) {
    FuzzScene fuzz;
    fuzz.build(0x5EED02u);

    Rng rng(0x9876u);
    std::size_t hits = 0;

    const int kSweeps = 10000;
    for (int i = 0; i < kSweeps; ++i) {
        const Vec3 half(rng.range(0.1f, 0.9f), rng.range(0.1f, 0.9f), rng.range(0.1f, 0.9f));
        const Vec3 from(rng.range(-14.0f, 14.0f), rng.range(2.0f, 14.0f), rng.range(-14.0f, 14.0f));
        const Vec3 to = from + Vec3(rng.range(-10.0f, 10.0f), rng.range(-14.0f, 2.0f),
                                    rng.range(-10.0f, 10.0f));

        Aabb box;
        box.min = from - half;
        box.max = from + half;

        const SweepHit accelerated = fuzz.physics.sweep_aabb(box, from, to);
        const SweepHit reference = fuzz.reference_sweep(half, from, to);

        ASSERT_EQ(accelerated.hit, reference.hit) << "sweep " << i;
        if (reference.hit) {
            ++hits;
            EXPECT_NEAR(accelerated.t, reference.t, 1.0e-4f) << "sweep " << i;
            EXPECT_EQ(accelerated.entity, reference.entity) << "sweep " << i;
        }
    }

    EXPECT_GT(hits, 0u);
}

TEST(QueryFuzz, OverlapSphereMatchesBruteForce) {
    FuzzScene fuzz;
    fuzz.build(0x5EED03u);

    Rng rng(0x2468u);
    for (int i = 0; i < 2000; ++i) {
        const Vec3 center(rng.range(-10.0f, 10.0f), rng.range(0.0f, 10.0f),
                          rng.range(-10.0f, 10.0f));
        const float radius = rng.range(0.1f, 2.5f);

        std::vector<EntityId> accelerated = fuzz.physics.overlap_sphere(center, radius);
        std::vector<EntityId> reference;
        for (const EntityId id : fuzz.world.entities()) {
            const Collider* collider = fuzz.world.get_collider(id);
            if (collider == nullptr) {
                continue;
            }
            Vec3 n;
            float pen = 0.0f;
            bool hit = false;
            if (collider->shape == Collider::Sphere) {
                hit = sphere_sphere(center, radius, fuzz.physics.position_of(id), collider->radius,
                                    n, pen);
            } else {
                hit = sphere_obb(center, radius, fuzz.physics.world_obb(id), n, pen);
            }
            if (hit) {
                reference.push_back(id);
            }
        }

        ASSERT_EQ(accelerated.size(), reference.size()) << "query " << i;
        for (std::size_t k = 0; k < accelerated.size(); ++k) {
            EXPECT_EQ(accelerated[k], reference[k]) << "query " << i;
        }
    }
}
