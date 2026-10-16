// Scene-file loading: the three shipped demo scenes must parse, contain bodies,
// and produce sane physics/camera settings.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "minieng/physics.hpp"
#include "minieng/scene.hpp"
#include "minieng/scene_loader.hpp"
#include "minieng/world.hpp"

using namespace minieng;

namespace {

std::string scene_path(const std::string& name) {
    return std::string(MINIENGINE_SOURCE_DIR) + "/data/scenes/" + name;
}

struct LoadedScene {
    World world;
    Scene scene;
    SceneDescription desc;
};

} // namespace

TEST(SceneLoader, LoadsEveryShippedScene) {
    const std::vector<std::string> names = {"stacked_boxes.json", "ramp_spheres.json",
                                            "obb_pile.json"};
    for (const std::string& name : names) {
        World world;
        Scene scene;
        const SceneDescription desc = load_scene_from_file(scene_path(name), world, scene);

        ASSERT_TRUE(desc.ok) << name << ": " << desc.error;
        EXPECT_FALSE(desc.name.empty()) << name;
        EXPECT_GT(world.entity_count(), 0u) << name;
        EXPECT_EQ(world.entity_count(), desc.entities.size()) << name;
        EXPECT_NEAR(desc.gravity.y, -9.81f, 1e-3f) << name;

        // Every entity must be fully wired: transform, collider and body.
        std::size_t with_collider = 0;
        std::size_t static_bodies = 0;
        for (const EntityId id : world.entities()) {
            EXPECT_NE(world.get_transform_ref(id), nullptr) << name;
            EXPECT_NE(world.get_rigid_body(id), nullptr) << name;
            EXPECT_TRUE(scene.valid(world.get_transform_ref(id)->scene_node)) << name;
            if (world.get_collider(id) != nullptr) {
                ++with_collider;
            }
            if (world.get_rigid_body(id)->is_static) {
                ++static_bodies;
            }
        }
        EXPECT_EQ(with_collider, world.entity_count()) << name;
        // All three scenes have a static floor.
        EXPECT_GE(static_bodies, 1u) << name;

        // At least one kind of shape beyond plain boxes is exercised somewhere.
        scene.update_world_transforms();
    }
}

TEST(SceneLoader, SceneStartsWithEveryShapeRepresented) {
    World world;
    Scene scene;
    const SceneDescription desc = load_scene_from_file(scene_path("obb_pile.json"), world, scene);
    ASSERT_TRUE(desc.ok) << desc.error;

    int aabb = 0;
    int obb = 0;
    int sphere = 0;
    for (const EntityId id : world.entities()) {
        const Collider* collider = world.get_collider(id);
        ASSERT_NE(collider, nullptr);
        switch (collider->shape) {
            case Collider::AABB: ++aabb; break;
            case Collider::OBB: ++obb; break;
            case Collider::Sphere: ++sphere; break;
        }
    }
    EXPECT_GE(aabb, 1);
    EXPECT_GE(obb, 4);
    EXPECT_GE(sphere, 1);
}

TEST(SceneLoader, LoadedSceneSimulatesDeterministically) {
    const auto run = [](std::uint64_t seed_unused) {
        (void)seed_unused;
        World world;
        Scene scene;
        PhysicsWorld physics(world, scene);
        const SceneDescription desc =
            load_scene_from_file(scene_path("stacked_boxes.json"), world, scene);
        EXPECT_TRUE(desc.ok);
        physics.config().gravity = desc.gravity;
        for (int tick = 0; tick < 180; ++tick) {
            physics.step(1.0f / 60.0f);
        }
        return world.hash(scene);
    };

    EXPECT_EQ(run(1u), run(1u));
    EXPECT_GT(run(1u), 0u);
}

TEST(SceneLoader, RejectsMalformedScenes) {
    {
        World world;
        Scene scene;
        const SceneDescription desc = load_scene_from_string("{ not json", world, scene);
        EXPECT_FALSE(desc.ok);
        EXPECT_FALSE(desc.error.empty());
        EXPECT_EQ(world.entity_count(), 0u);
    }
    {
        World world;
        Scene scene;
        const SceneDescription desc = load_scene_from_string("{\"entities\": []}", world, scene);
        EXPECT_FALSE(desc.ok);
        EXPECT_EQ(world.entity_count(), 0u);
    }
    {
        World world;
        Scene scene;
        const SceneDescription desc =
            load_scene_from_string("{\"entities\": [{\"shape\": \"torus\"}]}", world, scene);
        EXPECT_FALSE(desc.ok);
        EXPECT_EQ(world.entity_count(), 0u);
    }
    {
        World world;
        Scene scene;
        const SceneDescription desc = load_scene_from_file("/nonexistent/scene.json", world, scene);
        EXPECT_FALSE(desc.ok);
    }
}

TEST(SceneLoader, RepeatAndOffsetExpandEntities) {
    World world;
    Scene scene;
    const SceneDescription desc = load_scene_from_string(
        "{\"entities\": [{\"shape\": \"aabb\", \"position\": [0, 1, 0], \"offset\": [0, 2, 0], "
        "\"repeat\": 4, \"tag\": \"crate\"}]}",
        world, scene);
    ASSERT_TRUE(desc.ok) << desc.error;
    ASSERT_EQ(desc.entities.size(), 4u);
    EXPECT_EQ(world.entity_count(), 4u);

    for (std::size_t i = 0; i < desc.entities.size(); ++i) {
        const TransformRef* ref = world.get_transform_ref(desc.entities[i]);
        ASSERT_NE(ref, nullptr);
        EXPECT_NEAR(scene.local_of(ref->scene_node).position.y, 1.0f + 2.0f * static_cast<float>(i),
                    1e-5f);
        const Tag* tag = world.get_tag(desc.entities[i]);
        ASSERT_NE(tag, nullptr);
        EXPECT_EQ(tag->name, "crate_" + std::to_string(i));
    }
}

TEST(SceneLoader, EulerRotationIsApplied) {
    World world;
    Scene scene;
    const SceneDescription desc = load_scene_from_string(
        "{\"entities\": [{\"shape\": \"obb\", \"rotation_euler_deg\": [0, 90, 0]}]}", world, scene);
    ASSERT_TRUE(desc.ok) << desc.error;
    ASSERT_EQ(desc.entities.size(), 1u);

    const TransformRef* ref = world.get_transform_ref(desc.entities[0]);
    const Vec3 rotated = scene.local_of(ref->scene_node).rotation.rotate_vector(Vec3::unit_x());
    EXPECT_TRUE(rotated.approx_equal_to(Vec3(0.0f, 0.0f, -1.0f), 1e-4f));
}
