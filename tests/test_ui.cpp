// UI layer tests: layout parsing/layout math, bindings, and focus navigation.
// Nothing here needs a window or ImGui.

#include <gtest/gtest.h>

#include <set>
#include <string>
#include <vector>

#include "minieng/physics.hpp"
#include "minieng/scene.hpp"
#include "minieng/ui/bindings.hpp"
#include "minieng/ui/layout.hpp"
#include "minieng/ui/navigation.hpp"
#include "minieng/world.hpp"

using namespace minieng;
using namespace minieng::ui;

namespace {

std::string data_path(const std::string& relative) {
    return std::string(MINIENGINE_SOURCE_DIR) + "/" + relative;
}

struct UiFixture {
    World world;
    Scene scene;
    PhysicsConfig config;
    SimControl control;
    BindingTable table;
    EntityId selected = kInvalidEntity;

    UiFixture() {
        register_simulation_bindings(table, world, scene, config, selected, control);
    }

    EntityId spawn(const Vec3& pos, float restitution = 0.4f) {
        const EntityId id = world.create_entity();
        const int node = scene.create_node(-1, id);
        Transform t;
        t.position = pos;
        scene.set_local(node, t);
        world.add_transform_ref(id, node);
        Collider c;
        c.shape = Collider::Sphere;
        c.radius = 0.5f;
        world.add_collider(id, c);
        RigidBody b;
        b.restitution = restitution;
        world.add_rigid_body(id, b);
        scene.update_world_transforms();
        selected = id;
        return id;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------

TEST(UiLayout, ParsesRealLayoutFile) {
    std::string error;
    const Layout layout = Layout::load(data_path("data/ui/main.ui.json"), &error);
    ASSERT_TRUE(error.empty()) << error;

    EXPECT_EQ(layout.widget_count(), 17u);
    EXPECT_EQ(layout.focusable_widgets().size(), 10u);
    EXPECT_NE(layout.find("inspector"), nullptr);
    EXPECT_NE(layout.find("gravity"), nullptr);
    EXPECT_EQ(layout.find("does_not_exist"), nullptr);
}

TEST(UiLayout, ComputesAnchoredAndStackedRects) {
    Layout layout = Layout::load(data_path("data/ui/main.ui.json"));
    layout.compute_layout(1280.0f, 720.0f);

    // Root frame is inset by the document padding of 10.
    const Widget* inspector = layout.find("inspector");
    ASSERT_NE(inspector, nullptr);
    EXPECT_FLOAT_EQ(inspector->rect.x, 10.0f);
    EXPECT_FLOAT_EQ(inspector->rect.y, 10.0f);
    EXPECT_FLOAT_EQ(inspector->rect.w, 300.0f);
    EXPECT_FLOAT_EQ(inspector->rect.h, 400.0f);

    // Bottom-left anchor hangs off the bottom of the root frame.
    const Widget* controls = layout.find("controls");
    ASSERT_NE(controls, nullptr);
    EXPECT_FLOAT_EQ(controls->rect.x, 10.0f);
    EXPECT_FLOAT_EQ(controls->rect.y, 550.0f);
    EXPECT_FLOAT_EQ(controls->rect.h, 160.0f);

    // Top-right anchor.
    const Widget* hud = layout.find("hud");
    ASSERT_NE(hud, nullptr);
    EXPECT_FLOAT_EQ(hud->rect.x, 970.0f);
    EXPECT_FLOAT_EQ(hud->rect.y, 10.0f);

    // Column stacking inside the inspector: content starts at (10+8, 10+8) and
    // each row is (label height 22) + spacing 6.
    const Widget* title = layout.find("inspector_title");
    ASSERT_NE(title, nullptr);
    EXPECT_FLOAT_EQ(title->rect.x, 18.0f);
    EXPECT_FLOAT_EQ(title->rect.y, 18.0f);
    EXPECT_FLOAT_EQ(title->rect.w, 284.0f);

    const Widget* pos_x = layout.find("pos_x");
    ASSERT_NE(pos_x, nullptr);
    EXPECT_FLOAT_EQ(pos_x->rect.y, 46.0f);
    EXPECT_FLOAT_EQ(pos_x->rect.h, 22.0f);

    const Widget* gravity = layout.find("gravity");
    ASSERT_NE(gravity, nullptr);
    EXPECT_FLOAT_EQ(gravity->rect.y, 186.0f);

    // Stacked children never overlap vertically.
    EXPECT_LT(pos_x->rect.bottom(), gravity->rect.y);
    EXPECT_GT(gravity->rect.bottom(), gravity->rect.y);
}

TEST(UiLayout, EditingTheJsonChangesTheTreeWithNoCodeChange) {
    const Layout base = Layout::load(data_path("data/ui/main.ui.json"));
    Layout extended = Layout::load(data_path("data/ui/alt.ui.json"));

    EXPECT_EQ(base.widget_count(), 17u);
    EXPECT_EQ(extended.widget_count(), 20u);
    EXPECT_EQ(extended.focusable_widgets().size(), 12u);

    // The extra panel only exists in the edited file.
    EXPECT_EQ(base.find("debug"), nullptr);
    ASSERT_NE(extended.find("debug"), nullptr);
    ASSERT_NE(extended.find("use_sap"), nullptr);

    extended.compute_layout(1280.0f, 720.0f);
    const Widget* debug = extended.find("debug");
    EXPECT_FLOAT_EQ(debug->rect.y, 430.0f);  // anchor offset 420 + root padding 10
    EXPECT_FLOAT_EQ(debug->rect.h, 150.0f);
}

TEST(UiLayout, ReportsMalformedJson) {
    std::string error;
    const Layout layout = Layout::parse("{ this is not json }", &error);
    EXPECT_FALSE(error.empty());
    EXPECT_EQ(layout.widget_count(), 0u);

    std::string error2;
    Layout::parse("{ \"nope\": [] }", &error2);
    EXPECT_FALSE(error2.empty());
}

// ---------------------------------------------------------------------------
// Bindings
// ---------------------------------------------------------------------------

TEST(UiBindings, SliderWritesThroughToPhysicsConfig) {
    UiFixture fx;
    EXPECT_NEAR(fx.table.get_float("physics.gravity.y"), -9.81f, 1e-4f);

    fx.table.set_float("physics.gravity.y", -20.0f);
    EXPECT_NEAR(fx.config.gravity.y, -20.0f, 1e-5f);

    // Values are clamped to the binding's declared range.
    fx.table.set_float("physics.gravity.y", -1000.0f);
    EXPECT_FLOAT_EQ(fx.config.gravity.y, -30.0f);

    fx.table.set_float("physics.damping", 0.25f);
    EXPECT_NEAR(fx.config.linear_damping, 0.25f, 1e-6f);
}

TEST(UiBindings, InspectorFieldsWriteToSelectedEntity) {
    UiFixture fx;
    const EntityId id = fx.spawn(Vec3(1.0f, 2.0f, 3.0f), 0.5f);

    EXPECT_NEAR(fx.table.get_float("selected.position.y"), 2.0f, 1e-6f);
    fx.table.set_float("selected.position.y", 7.5f);
    const TransformRef* ref = fx.world.get_transform_ref(id);
    EXPECT_NEAR(fx.scene.local_of(ref->scene_node).position.y, 7.5f, 1e-6f);

    // Per-body values take priority when an entity is selected.
    EXPECT_NEAR(fx.table.get_float("physics.restitution"), 0.5f, 1e-6f);
    fx.table.set_float("physics.restitution", 0.9f);
    EXPECT_NEAR(fx.world.get_rigid_body(id)->restitution, 0.9f, 1e-6f);

    // Read-only stats cannot be written.
    EXPECT_TRUE(fx.table.is_read_only("stats.fps"));
    EXPECT_FALSE(fx.table.is_read_only("physics.gravity.y"));
    fx.table.set_float("stats.fps", 123.0f);
    EXPECT_FLOAT_EQ(fx.control.fps, 0.0f);
}

TEST(UiBindings, ActionsAndToggles) {
    UiFixture fx;
    EXPECT_FALSE(fx.control.paused);

    fx.table.set_bool("sim.pause", true);
    EXPECT_TRUE(fx.control.paused);
    EXPECT_TRUE(fx.table.toggle("sim.pause"));
    EXPECT_FALSE(fx.control.paused);

    EXPECT_TRUE(fx.table.is_action("sim.step"));
    fx.table.activate("sim.step");
    EXPECT_TRUE(fx.control.step_requested);
    fx.table.activate("sim.reset");
    EXPECT_TRUE(fx.control.reset_requested);

    // Unknown names are inert rather than fatal.
    EXPECT_FALSE(fx.table.activate("nope"));
    EXPECT_FALSE(fx.table.has("nope"));
}

// ---------------------------------------------------------------------------
// Focus navigation
// ---------------------------------------------------------------------------

TEST(UiNavigation, TabOrderVisitsEveryWidgetAndWraps) {
    Layout layout = Layout::load(data_path("data/ui/main.ui.json"));
    layout.compute_layout(1280.0f, 720.0f);

    UiContext ctx;
    ctx.set_items_from_layout(layout);
    ctx.clear_focus();
    ASSERT_EQ(ctx.size(), 10u);

    const std::string first = ctx.items().front().id;

    std::set<std::string> visited;
    ctx.move_focus(FocusDir::Next);
    visited.insert(ctx.focused_id());
    for (std::size_t i = 1; i < ctx.size(); ++i) {
        ctx.move_focus(FocusDir::Next);
        visited.insert(ctx.focused_id());
    }
    EXPECT_EQ(visited.size(), ctx.size()) << "tab order must reach every focusable widget";
    EXPECT_EQ(ctx.focused_id(), ctx.items().back().id);

    // One more step wraps back to the beginning.
    ctx.move_focus(FocusDir::Next);
    EXPECT_EQ(ctx.focused_id(), first);

    // Backwards from the first wraps to the last.
    ctx.focus(first);
    ctx.move_focus(FocusDir::Prev);
    EXPECT_EQ(ctx.focused_id(), ctx.items().back().id);

    // Tab order matches the depth-first layout order.
    EXPECT_EQ(ctx.items().front().id, "pos_x");
    EXPECT_EQ(ctx.items().back().id, "wireframe");
}

TEST(UiNavigation, SpatialMovementAndWrapping) {
    UiContext ctx;
    const std::vector<FocusItem> items = {
        {"a", "", Rect{0.0f, 0.0f, 100.0f, 20.0f}},
        {"b", "", Rect{200.0f, 0.0f, 100.0f, 20.0f}},
        {"c", "", Rect{0.0f, 100.0f, 100.0f, 20.0f}},
        {"d", "", Rect{200.0f, 100.0f, 100.0f, 20.0f}},
    };
    ctx.set_items(items);

    ctx.clear_focus();
    // First move with no focus lands on the first item.
    EXPECT_TRUE(ctx.move_focus(FocusDir::Right));
    EXPECT_EQ(ctx.focused_id(), "a");

    EXPECT_TRUE(ctx.move_focus(FocusDir::Right));
    EXPECT_EQ(ctx.focused_id(), "b");
    ctx.move_focus(FocusDir::Down);
    EXPECT_EQ(ctx.focused_id(), "d");
    ctx.move_focus(FocusDir::Left);
    EXPECT_EQ(ctx.focused_id(), "c");
    ctx.move_focus(FocusDir::Up);
    EXPECT_EQ(ctx.focused_id(), "a");

    // Wrapping: there is nothing to the right of the rightmost item, so focus
    // wraps around to the leftmost one.
    ctx.focus("b");
    ctx.move_focus(FocusDir::Right);
    EXPECT_EQ(ctx.focused_id(), "a");
}

TEST(UiNavigation, ActivateFiresBoundAction) {
    UiFixture fx;

    std::vector<FocusItem> items = {
        {"pause", "sim.pause", Rect{0, 0, 50, 20}},
        {"step", "sim.step", Rect{0, 30, 50, 20}},
    };
    UiContext ctx;
    ctx.set_items(items);
    ctx.set_bindings(&fx.table);

    ASSERT_TRUE(ctx.focus("pause"));
    EXPECT_TRUE(ctx.activate());
    EXPECT_TRUE(fx.control.paused);
    EXPECT_TRUE(ctx.activate());
    EXPECT_FALSE(fx.control.paused);

    ASSERT_TRUE(ctx.focus("step"));
    EXPECT_TRUE(ctx.activate());
    EXPECT_TRUE(fx.control.step_requested);

    // An unbound item does nothing.
    ctx.set_items({{"plain", "", Rect{0, 0, 10, 10}}});
    ctx.focus("plain");
    EXPECT_FALSE(ctx.activate());
}

TEST(UiNavigation, FocusIsDroppedWhenItemDisappears) {
    UiContext ctx;
    ctx.set_items({{"a", "", Rect{0, 0, 10, 10}}, {"b", "", Rect{0, 20, 10, 10}}});
    ASSERT_TRUE(ctx.focus("b"));
    ctx.set_items({{"a", "", Rect{0, 0, 10, 10}}});
    EXPECT_FALSE(ctx.has_focus());
}

TEST(UiLayout, ComputedRectsRespondToViewportSize) {
    Layout layout = Layout::load(data_path("data/ui/main.ui.json"));

    layout.compute_layout(1280.0f, 720.0f);
    const float wide_hud_x = layout.find("hud")->rect.x;

    layout.compute_layout(800.0f, 600.0f);
    const float narrow_hud_x = layout.find("hud")->rect.x;
    EXPECT_LT(narrow_hud_x, wide_hud_x);

    // The bottom-anchored panel moves up as the viewport shrinks.
    EXPECT_FLOAT_EQ(layout.find("controls")->rect.y, 430.0f);
}
