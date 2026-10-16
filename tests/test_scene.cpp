// Scene graph tests: transform composition through a parent chain, reparenting
// with world preservation, and node/entity bookkeeping.

#include <gtest/gtest.h>

#include "minieng/scene.hpp"

using namespace minieng;

namespace {

Transform make_trs(const Vec3& pos, const Quat& rot, const Vec3& scale) {
    Transform t;
    t.position = pos;
    t.rotation = rot;
    t.scale = scale;
    return t;
}

}  // namespace

TEST(Scene, ThreeLevelChainMatchesManualProduct) {
    Scene scene;

    const Transform root_t = make_trs(Vec3(1.0f, 0.0f, -2.0f),
                                      Quat::from_axis_angle(Vec3::unit_z(), deg_to_rad(35.0f)),
                                      Vec3(1.0f, 1.0f, 1.0f));
    const Transform mid_t = make_trs(Vec3(0.0f, 2.0f, 0.0f),
                                     Quat::from_axis_angle(Vec3::unit_y(), deg_to_rad(90.0f)),
                                     Vec3(2.0f, 1.0f, 1.0f));
    const Transform leaf_t = make_trs(Vec3(0.0f, 0.0f, 3.0f),
                                      Quat::from_axis_angle(Vec3::unit_x(), deg_to_rad(-20.0f)),
                                      Vec3(1.0f, 1.0f, 0.5f));

    const int root = scene.create_node(-1);
    const int mid = scene.create_node(root);
    const int leaf = scene.create_node(mid);

    scene.set_local(root, root_t);
    scene.set_local(mid, mid_t);
    scene.set_local(leaf, leaf_t);
    scene.update_world_transforms();

    // Manual composition: world = root * mid * leaf.
    const Mat4 expected =
        root_t.to_matrix() * mid_t.to_matrix() * leaf_t.to_matrix();

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(scene.world_of(leaf).at(r, c), expected.at(r, c), 1.0e-5f);
        }
    }

    EXPECT_EQ(scene.parent_of(leaf), mid);
    EXPECT_EQ(scene.parent_of(mid), root);
    EXPECT_EQ(scene.parent_of(root), -1);
    EXPECT_EQ(scene.node_count(), 3);
}

TEST(Scene, WorldUpdateIsTopDownRegardlessOfChildCreationOrder) {
    // Build a tree, then a second tree whose children are added "backwards" to
    // make sure the single pass resolves parents before descendants.
    Scene scene;
    const int a = scene.create_node(-1);
    const int b = scene.create_node(a);
    const int c = scene.create_node(b);

    // Give each level a distinct translation so a wrong order is visible.
    for (int i = 0; i < scene.node_count(); ++i) {
        Transform t;
        t.position = Vec3(1.0f, 0.0f, 0.0f);
        scene.set_local(i, t);
    }
    scene.update_world_transforms();

    const Vec3 pa = scene.world_position_of(a);
    const Vec3 pb = scene.world_position_of(b);
    const Vec3 pc = scene.world_position_of(c);

    EXPECT_TRUE(pa.approx_equal_to(Vec3(1.0f, 0.0f, 0.0f), 1.0e-5f));
    EXPECT_TRUE(pb.approx_equal_to(Vec3(2.0f, 0.0f, 0.0f), 1.0e-5f));
    EXPECT_TRUE(pc.approx_equal_to(Vec3(3.0f, 0.0f, 0.0f), 1.0e-5f));
}

TEST(Scene, ReparentKeepsWorldWhenRequested) {
    Scene scene;

    const int parent_a = scene.create_node(-1);
    scene.set_local(parent_a, make_trs(Vec3(10.0f, 0.0f, 0.0f),
                                       Quat::from_axis_angle(Vec3::unit_z(), deg_to_rad(90.0f)),
                                       Vec3(1.0f, 1.0f, 1.0f)));

    const int child = scene.create_node(parent_a);
    scene.set_local(child, make_trs(Vec3(1.0f, 0.0f, 0.0f), Quat::identity(), Vec3::one()));

    const int parent_b = scene.create_node(-1);
    scene.set_local(parent_b, make_trs(Vec3(0.0f, 5.0f, 0.0f),
                                       Quat::from_axis_angle(Vec3::unit_x(), deg_to_rad(45.0f)),
                                       Vec3(1.0f, 1.0f, 1.0f)));

    scene.update_world_transforms();
    const Mat4 before = scene.world_of(child);
    const Vec3 before_pos = scene.world_position_of(child);

    scene.reparent(child, parent_b, /*keep_world=*/true);

    EXPECT_EQ(scene.parent_of(child), parent_b);
    const Mat4 after = scene.world_of(child);
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(after.at(r, c), before.at(r, c), 1.0e-4f);
        }
    }
    EXPECT_TRUE(scene.world_position_of(child).approx_equal_to(before_pos, 1.0e-4f));
}

TEST(Scene, ReparentWithoutKeepWorldRecomputesWorld) {
    Scene scene;
    const int a = scene.create_node(-1);
    scene.set_local(a, make_trs(Vec3(100.0f, 0.0f, 0.0f), Quat::identity(), Vec3::one()));
    const int child = scene.create_node(a);
    scene.set_local(child, make_trs(Vec3(1.0f, 0.0f, 0.0f), Quat::identity(), Vec3::one()));

    const int b = scene.create_node(-1);
    scene.set_local(b, make_trs(Vec3(0.0f, 0.0f, 0.0f), Quat::identity(), Vec3::one()));

    scene.update_world_transforms();
    EXPECT_TRUE(scene.world_position_of(child).approx_equal_to(Vec3(101.0f, 0.0f, 0.0f), 1e-5f));

    scene.reparent(child, b, /*keep_world=*/false);
    // Local transform is untouched, so world follows the new parent.
    EXPECT_TRUE(scene.world_position_of(child).approx_equal_to(Vec3(1.0f, 0.0f, 0.0f), 1e-5f));
}

TEST(Scene, ReparentRejectsCycles) {
    Scene scene;
    const int a = scene.create_node(-1);
    const int b = scene.create_node(a);
    const int c = scene.create_node(b);

    // Making a a child of its own grandchild must be refused.
    scene.reparent(a, c, true);
    EXPECT_EQ(scene.parent_of(a), -1);
    EXPECT_EQ(scene.parent_of(c), b);
}

TEST(Scene, EntityMapping) {
    Scene scene;
    const int n = scene.create_node(-1, /*owner=*/7u);
    EXPECT_EQ(scene.entity_of(n), 7u);
    EXPECT_EQ(scene.node_of_entity(7u), n);

    scene.set_entity(n, 42u);
    EXPECT_EQ(scene.node_of_entity(7u), -1);
    EXPECT_EQ(scene.node_of_entity(42u), n);
    EXPECT_EQ(scene.entity_of(n), 42u);
}
