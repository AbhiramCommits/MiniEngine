// MiniEngine sandbox: a thin GLFW + Dear ImGui front-end over minieng_core.
//
// The app owns no simulation logic. It builds a scene through the same World /
// Scene / PhysicsWorld / FixedLoop APIs the tests use, renders the parsed UI
// layout with ImGui, projects the scene with the engine's own Mat4 camera, and
// lets the keyboard / gamepad drive focus traversal.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "minieng/loop.hpp"
#include "minieng/physics.hpp"
#include "minieng/profiler.hpp"
#include "minieng/rng.hpp"
#include "minieng/scene.hpp"
#include "minieng/scene_loader.hpp"
#include "minieng/ui/bindings.hpp"
#include "minieng/ui/layout.hpp"
#include "minieng/ui/navigation.hpp"
#include "minieng/world.hpp"

using namespace minieng;
using namespace minieng::ui;

namespace {

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

struct Camera {
    Vec3 eye = Vec3(0.0f, 10.0f, 17.0f);
    Vec3 target = Vec3(0.0f, 1.5f, 0.0f);
    float fov_y = deg_to_rad(55.0f);
    float near_z = 0.1f;
    float far_z = 400.0f;

    Vec3 forward = Vec3(0.0f, 0.0f, -1.0f);
    Vec3 right = Vec3(1.0f, 0.0f, 0.0f);
    Vec3 up = Vec3(0.0f, 1.0f, 0.0f);

    void update_basis() {
        forward = (target - eye).normalized();
        right = forward.cross(Vec3::unit_y()).normalized();
        if (right.length_squared() < kEpsilonSq) {
            right = Vec3::unit_x();
        }
        up = right.cross(forward);
    }

    Mat4 view() const { return Mat4::look_at(eye, target, Vec3::unit_y()); }
    Mat4 projection(float aspect) const { return Mat4::perspective(fov_y, aspect, near_z, far_z); }

    // Ray direction through a normalized device coordinate.
    Vec3 ray_direction(float ndc_x, float ndc_y, float aspect) const {
        const float t = std::tan(fov_y * 0.5f);
        return (forward + right * (ndc_x * aspect * t) + up * (ndc_y * t)).normalized();
    }
};

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

struct AppState {
    World world;
    Scene scene;
    PhysicsWorld physics{world, scene};
    FixedLoop loop{1.0f / 60.0f, 8};

    SimControl control;
    BindingTable bindings;
    Layout layout;
    UiContext ui;
    Camera camera;

    EntityId selected = kInvalidEntity;
    std::uint64_t seed = 1337u;
    std::string scene_path; // empty == built-in demo scene

    float frame_ms = 0.0f;
    float queries_per_sec = 0.0f;
    double last_time = 0.0;
};

// ---------------------------------------------------------------------------
// Scene construction
// ---------------------------------------------------------------------------

EntityId add_body(AppState& app, const Vec3& position, const Quat& rotation,
                  const Collider& collider, const RigidBody& body) {
    const EntityId id = app.world.create_entity();
    const int node = app.scene.create_node(-1, id);
    Transform t;
    t.position = position;
    t.rotation = rotation;
    app.scene.set_local(node, t);
    app.world.add_transform_ref(id, node);
    app.world.add_collider(id, collider);
    app.world.add_rigid_body(id, body);
    return id;
}

EntityId add_static_box(AppState& app, const Vec3& position, const Vec3& half) {
    Collider c;
    c.shape = Collider::AABB;
    c.half_extents = half;
    RigidBody b;
    b.is_static = true;
    b.mass = 0.0f;
    b.restitution = 0.1f;
    b.friction = 0.6f;
    return add_body(app, position, Quat::identity(), c, b);
}

EntityId add_dynamic_box(AppState& app, const Vec3& position, const Vec3& half, float restitution) {
    Collider c;
    c.shape = Collider::AABB;
    c.half_extents = half;
    RigidBody b;
    b.mass = 1.0f;
    b.restitution = restitution;
    b.friction = 0.5f;
    return add_body(app, position, Quat::identity(), c, b);
}

EntityId add_sphere(AppState& app, const Vec3& position, float radius, float restitution) {
    Collider c;
    c.shape = Collider::Sphere;
    c.radius = radius;
    RigidBody b;
    b.mass = 0.6f;
    b.restitution = restitution;
    b.friction = 0.35f;
    return add_body(app, position, Quat::identity(), c, b);
}

EntityId add_obb(AppState& app, const Vec3& position, const Vec3& half, const Quat& rotation,
                 float restitution) {
    Collider c;
    c.shape = Collider::OBB;
    c.half_extents = half;
    RigidBody b;
    b.mass = 1.2f;
    b.restitution = restitution;
    b.friction = 0.5f;
    return add_body(app, position, rotation, c, b);
}

void build_default_scene(AppState& app, std::uint64_t seed) {
    const PhysicsConfig& cfg = app.physics.config();

    // Ground.
    add_static_box(app, Vec3(0.0f, -0.5f, 0.0f), Vec3(12.0f, 0.5f, 12.0f));

    // A stack of boxes.
    for (int i = 0; i < 5; ++i) {
        add_dynamic_box(app, Vec3(0.0f, 0.5f + 1.06f * static_cast<float>(i), 0.0f),
                        Vec3(0.5f, 0.5f, 0.5f), cfg.default_restitution);
    }

    // Rotated boxes are a good SAT/SAT-diagnostic demo.
    add_obb(app, Vec3(3.5f, 1.0f, 0.0f), Vec3(0.9f, 0.2f, 0.4f),
            Quat::from_axis_angle(Vec3::unit_z(), deg_to_rad(35.0f)), 0.3f);
    add_obb(app, Vec3(3.5f, 3.0f, 0.0f), Vec3(0.9f, 0.2f, 0.4f),
            Quat::from_axis_angle(Vec3::unit_x(), deg_to_rad(-50.0f)), 0.3f);

    Rng rng(seed);
    for (int i = 0; i < 10; ++i) {
        add_sphere(
            app, Vec3(rng.range(-6.0f, 6.0f), 3.0f + rng.range(0.0f, 7.0f), rng.range(-6.0f, 6.0f)),
            0.45f, 0.55f);
    }
    for (int i = 0; i < 8; ++i) {
        add_dynamic_box(
            app, Vec3(rng.range(-7.0f, 7.0f), 4.0f + rng.range(0.0f, 8.0f), rng.range(-7.0f, 7.0f)),
            Vec3(0.4f, 0.4f, 0.4f), 0.25f);
    }
}

// Clears the world and rebuilds it, either from --scene or from the built-in
// demo used when no scene file was given.
void build_scene(AppState& app, std::uint64_t seed) {
    app.world.clear();
    app.scene.clear();
    app.selected = kInvalidEntity;
    app.seed = seed;

    if (!app.scene_path.empty()) {
        const SceneDescription desc = load_scene_from_file(app.scene_path, app.world, app.scene);
        if (desc.ok) {
            app.physics.config().gravity = desc.gravity;
            app.camera.eye = desc.camera_eye;
            app.camera.target = desc.camera_target;
            app.camera.update_basis();
            std::printf("scene: %s (%zu entities)\n", desc.name.c_str(), desc.entities.size());
        } else {
            std::fprintf(stderr, "could not load scene '%s': %s\n", app.scene_path.c_str(),
                         desc.error.c_str());
        }
    } else {
        build_default_scene(app, seed);
    }

    app.scene.update_world_transforms();
    app.loop.reset();

    // Select the first dynamic body.
    for (const EntityId id : app.world.entities()) {
        const RigidBody* body = app.world.get_rigid_body(id);
        if (body != nullptr && !body->is_static) {
            app.selected = id;
            break;
        }
    }

    app.physics.build_broadphase();
}

// ---------------------------------------------------------------------------
// Simulation pipeline
// ---------------------------------------------------------------------------

void install_systems(AppState& app) {
    app.loop.clear_systems();
    app.loop.set_system(SystemStage::PhysicsIntegrate, [&app](float dt) {
        MINIENG_PROFILE("integrate");
        app.physics.integrate(dt);
    });
    app.loop.set_system(SystemStage::CollisionResolve, [&app](float dt) {
        std::vector<Pair> pairs;
        {
            MINIENG_PROFILE("broadphase");
            app.physics.build_broadphase();
            pairs = app.physics.broadphase_pairs();
        }
        std::vector<Contact> contacts;
        {
            MINIENG_PROFILE("narrowphase");
            contacts = app.physics.narrowphase(pairs);
        }
        {
            MINIENG_PROFILE("resolve");
            app.physics.resolve_contacts(contacts, dt);
        }
        app.control.pair_count = static_cast<int>(pairs.size());
        app.control.contact_count = static_cast<int>(contacts.size());
    });
    app.loop.set_system(SystemStage::SceneTransform, [&app](float) {
        MINIENG_PROFILE("transforms");
        app.scene.update_world_transforms();
    });
    app.loop.set_system(SystemStage::UiObserve, [&app](float) {
        app.control.ticks = app.loop.tick_count();
        app.control.body_count = static_cast<int>(app.world.entity_count());
    });
}

// ---------------------------------------------------------------------------
// Rendering helpers
// ---------------------------------------------------------------------------

bool project_point(const Mat4& view_proj, const Vec3& p, float width, float height, ImVec2& out) {
    const float x = view_proj.at(0, 0) * p.x + view_proj.at(0, 1) * p.y + view_proj.at(0, 2) * p.z +
                    view_proj.at(0, 3);
    const float y = view_proj.at(1, 0) * p.x + view_proj.at(1, 1) * p.y + view_proj.at(1, 2) * p.z +
                    view_proj.at(1, 3);
    const float w = view_proj.at(3, 0) * p.x + view_proj.at(3, 1) * p.y + view_proj.at(3, 2) * p.z +
                    view_proj.at(3, 3);
    if (w < 1.0e-4f) {
        return false;
    }
    const float ndc_x = x / w;
    const float ndc_y = y / w;
    if (ndc_x < -1.5f || ndc_x > 1.5f || ndc_y < -1.5f || ndc_y > 1.5f) {
        return false;
    }
    out = ImVec2((ndc_x * 0.5f + 0.5f) * width, (1.0f - (ndc_y * 0.5f + 0.5f)) * height);
    return true;
}

void draw_line_3d(ImDrawList* draw, const Mat4& vp, const Vec3& a, const Vec3& b, float w, float h,
                  ImU32 color) {
    ImVec2 pa;
    ImVec2 pb;
    if (project_point(vp, a, w, h, pa) && project_point(vp, b, w, h, pb)) {
        draw->AddLine(pa, pb, color, 1.2f);
    }
}

void draw_scene_lines(AppState& app, ImDrawList* draw, float width, float height) {
    const Mat4 view = app.camera.view();
    const Mat4 proj = app.camera.projection(width / (height > 1.0f ? height : 1.0f));
    const Mat4 view_proj = proj * view;

    // Ground grid.
    const ImU32 grid = IM_COL32(70, 80, 95, 160);
    for (int i = -12; i <= 12; ++i) {
        const float f = static_cast<float>(i);
        draw_line_3d(draw, view_proj, Vec3(f, 0.0f, -12.0f), Vec3(f, 0.0f, 12.0f), width, height,
                     grid);
        draw_line_3d(draw, view_proj, Vec3(-12.0f, 0.0f, f), Vec3(12.0f, 0.0f, f), width, height,
                     grid);
    }

    // World axes.
    draw_line_3d(draw, view_proj, Vec3::zero(), Vec3(2, 0, 0), width, height,
                 IM_COL32(220, 70, 70, 255));
    draw_line_3d(draw, view_proj, Vec3::zero(), Vec3(0, 2, 0), width, height,
                 IM_COL32(70, 220, 70, 255));
    draw_line_3d(draw, view_proj, Vec3::zero(), Vec3(0, 0, 2), width, height,
                 IM_COL32(70, 120, 240, 255));

    for (const EntityId id : app.world.entities()) {
        const Collider* collider = app.world.get_collider(id);
        if (collider == nullptr) {
            continue;
        }
        const Aabb box = app.physics.world_aabb(id);
        const Vec3 corners[8] = {
            Vec3(box.min.x, box.min.y, box.min.z), Vec3(box.max.x, box.min.y, box.min.z),
            Vec3(box.min.x, box.max.y, box.min.z), Vec3(box.max.x, box.max.y, box.min.z),
            Vec3(box.min.x, box.min.y, box.max.z), Vec3(box.max.x, box.min.y, box.max.z),
            Vec3(box.min.x, box.max.y, box.max.z), Vec3(box.max.x, box.max.y, box.max.z),
        };
        static const int edges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                         {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

        const bool is_static = app.physics.is_static(id);
        ImU32 color = is_static ? IM_COL32(120, 130, 145, 200) : IM_COL32(90, 200, 250, 220);
        if (id == app.selected) {
            color = IM_COL32(255, 210, 80, 255);
        }
        for (const auto& e : edges) {
            draw_line_3d(draw, view_proj, corners[e[0]], corners[e[1]], width, height, color);
        }
    }
}

// ---------------------------------------------------------------------------
// Declarative widget rendering (the layout file drives this)
// ---------------------------------------------------------------------------

void draw_widget(AppState& app, const Widget& widget) {
    if (widget.type == "panel") {
        ImGui::SetNextWindowPos(ImVec2(widget.rect.x, widget.rect.y), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(widget.rect.w, widget.rect.h), ImGuiCond_Always);
        const ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        if (ImGui::Begin(widget.id.c_str(), nullptr, flags)) {
            for (const Widget& child : widget.children) {
                draw_widget(app, child);
            }
        }
        ImGui::End();
        return;
    }

    ImGui::SetCursorScreenPos(ImVec2(widget.rect.x, widget.rect.y));

    const bool focused = app.ui.focused_id() == widget.id;
    if (focused) {
        ImGui::GetWindowDrawList()->AddRect(
            ImVec2(widget.rect.x - 2.0f, widget.rect.y - 2.0f),
            ImVec2(widget.rect.right() + 2.0f, widget.rect.bottom() + 2.0f),
            IM_COL32(255, 210, 80, 255), 3.0f, 0, 1.5f);
    }

    if (widget.type == "label") {
        const std::string value =
            widget.binding.empty() ? std::string() : app.bindings.get_string(widget.binding);
        if (value.empty()) {
            ImGui::TextUnformatted(widget.text.c_str());
        } else {
            ImGui::Text("%s: %s", widget.text.c_str(), value.c_str());
        }
    } else if (widget.type == "button") {
        if (ImGui::Button(widget.text.c_str(), ImVec2(widget.rect.w, widget.rect.h))) {
            app.bindings.activate(widget.binding);
        }
        if (focused) {
            ImGui::SetItemDefaultFocus();
        }
    } else if (widget.type == "slider") {
        float value = app.bindings.get_float(widget.binding);
        ImGui::SetNextItemWidth(widget.rect.w);
        if (ImGui::SliderFloat(widget.text.c_str(), &value, widget.min, widget.max)) {
            app.bindings.set_float(widget.binding, value);
        }
    } else if (widget.type == "checkbox") {
        bool checked = app.bindings.get_bool(widget.binding);
        if (ImGui::Checkbox(widget.text.c_str(), &checked)) {
            app.bindings.set_bool(widget.binding, checked);
        }
    } else if (widget.type == "list") {
        if (ImGui::BeginListBox(widget.id.c_str(), ImVec2(widget.rect.w, widget.rect.h))) {
            for (const EntityId id : app.world.entities()) {
                char label[64];
                std::snprintf(label, sizeof(label), "entity %u", id);
                const bool is_selected = id == app.selected;
                if (ImGui::Selectable(label, is_selected)) {
                    app.selected = id;
                }
            }
            ImGui::EndListBox();
        }
    }
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

void handle_navigation_input(AppState& app) {
    const ImGuiIO& io = ImGui::GetIO();
    (void)io;

    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow) ||
        ImGui::IsKeyPressed(ImGuiKey_GamepadDpadRight) || ImGui::IsKeyPressed(ImGuiKey_D)) {
        app.ui.move_focus(FocusDir::Right);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadLeft) ||
        ImGui::IsKeyPressed(ImGuiKey_A)) {
        app.ui.move_focus(FocusDir::Left);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadUp) ||
        ImGui::IsKeyPressed(ImGuiKey_W)) {
        app.ui.move_focus(FocusDir::Up);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow) || ImGui::IsKeyPressed(ImGuiKey_GamepadDpadDown) ||
        ImGui::IsKeyPressed(ImGuiKey_S)) {
        app.ui.move_focus(FocusDir::Down);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Tab)) {
        app.ui.move_focus(ImGui::GetIO().KeyShift ? FocusDir::Prev : FocusDir::Next);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_GamepadFaceDown) ||
        ImGui::IsKeyPressed(ImGuiKey_Space)) {
        app.ui.activate();
    }
}

void handle_mouse_selection(AppState& app, float width, float height) {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse || !ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        return;
    }
    const float ndc_x = 2.0f * (io.MousePos.x / width) - 1.0f;
    const float ndc_y = 1.0f - 2.0f * (io.MousePos.y / height);
    const Vec3 dir = app.camera.ray_direction(ndc_x, ndc_y, width / height);
    app.physics.build_broadphase();
    const RayHit hit = app.physics.raycast(app.camera.eye, dir, 500.0f);
    if (hit.hit) {
        app.selected = hit.entity;
    }
}

// ---------------------------------------------------------------------------
// Extra windows (entity list + live stats overlay)
// ---------------------------------------------------------------------------

void draw_entity_list(AppState& app) {
    ImGui::SetNextWindowPos(ImVec2(980.0f, 150.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280.0f, 260.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Entities")) {
        const RigidBody* body = app.world.get_rigid_body(app.selected);
        const TransformRef* ref = app.world.get_transform_ref(app.selected);
        ImGui::Text("selected: %u", app.selected);
        if (ref != nullptr && ref->scene_node >= 0) {
            const Transform& t = app.scene.local_of(ref->scene_node);
            ImGui::Text("pos  %.3f %.3f %.3f", t.position.x, t.position.y, t.position.z);
            if (body != nullptr) {
                ImGui::Text("vel  %.3f %.3f %.3f", body->velocity.x, body->velocity.y,
                            body->velocity.z);
            }
        }
        ImGui::Separator();
        if (ImGui::BeginChild("entity_scroll", ImVec2(0, 0), ImGuiChildFlags_None)) {
            for (const EntityId id : app.world.entities()) {
                char label[64];
                std::snprintf(label, sizeof(label), "entity %u", id);
                if (ImGui::Selectable(label, id == app.selected)) {
                    app.selected = id;
                }
            }
        }
        ImGui::EndChild();
    }
    ImGui::End();
}

void draw_stats_overlay(AppState& app) {
    ImGui::SetNextWindowPos(ImVec2(980.0f, 420.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(280.0f, 200.0f), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Profiler")) {
        const FrameStats frame = Profiler::global().frame_stats();
        ImGui::Text("frame   %.2f ms (p95 %.2f)", frame.mean_ms, frame.p95_ms);
        ImGui::Text("ticks   %llu", static_cast<unsigned long long>(app.control.ticks));
        ImGui::Text("bodies  %d", app.control.body_count);
        ImGui::Text("pairs   %d", app.control.pair_count);
        ImGui::Text("queries %.0f /s", static_cast<double>(app.queries_per_sec));
        ImGui::Separator();
        static const char* kSystems[] = {"integrate", "broadphase", "narrowphase", "resolve",
                                         "transforms"};
        for (const char* name : kSystems) {
            const FrameStats s = Profiler::global().stats(name);
            ImGui::Text("%-12s %6.3f ms", name, s.mean_ms);
        }
    }
    ImGui::End();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

struct Options {
    std::string ui_path = std::string(MINIENGINE_SOURCE_DIR) + "/data/ui/main.ui.json";
    std::string scene_path; // empty == built-in demo scene
    std::uint64_t seed = 1337u;
    int width = 1280;
    int height = 720;
    int max_frames = 0; // 0 == run until the window is closed
};

Options parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&](std::string& out) {
            if (i + 1 < argc) {
                out = argv[++i];
            }
        };
        if (arg == "--ui") {
            next(options.ui_path);
        } else if (arg == "--scene") {
            next(options.scene_path);
        } else if (arg == "--frames") {
            std::string value;
            next(value);
            options.max_frames = std::atoi(value.c_str());
        } else if (arg == "--seed") {
            std::string value;
            next(value);
            options.seed = static_cast<std::uint64_t>(std::strtoull(value.c_str(), nullptr, 10));
        } else if (arg == "--width") {
            std::string value;
            next(value);
            options.width = std::atoi(value.c_str());
        } else if (arg == "--height") {
            std::string value;
            next(value);
            options.height = std::atoi(value.c_str());
        } else if (arg == "--help" || arg == "-h") {
            std::printf(
                "usage: minieng_sandbox [--scene PATH] [--ui PATH] [--seed N] [--frames N] "
                "[--width W] [--height H]\n");
            std::exit(0);
        }
    }
    return options;
}

void glfw_error_callback(int error, const char* description) {
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description);
}

} // namespace

int main(int argc, char** argv) {
    const Options options = parse_args(argc, argv);

    glfwSetErrorCallback(glfw_error_callback);
    if (glfwInit() != GLFW_TRUE) {
        std::fprintf(stderr, "failed to initialise GLFW\n");
        return 1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

    GLFWwindow* window =
        glfwCreateWindow(options.width, options.height, "MiniEngine Sandbox", nullptr, nullptr);
    if (window == nullptr) {
        std::fprintf(stderr, "failed to create a window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150");

    AppState app;
    app.camera.update_basis();
    app.scene_path = options.scene_path;

    std::string error;
    app.layout = Layout::load(options.ui_path, &error);
    if (!error.empty()) {
        std::fprintf(stderr, "layout error: %s\n", error.c_str());
    }
    app.layout.compute_layout(static_cast<float>(options.width),
                              static_cast<float>(options.height));

    install_systems(app);
    build_scene(app, options.seed);
    register_simulation_bindings(app.bindings, app.world, app.scene, app.physics.config(),
                                 app.selected, app.control);
    app.ui.set_bindings(&app.bindings);
    app.ui.set_items_from_layout(app.layout);

    app.last_time = glfwGetTime();
    int frame_count = 0;

    while (glfwWindowShouldClose(window) == 0) {
        glfwPollEvents();

        int display_w = 0;
        int display_h = 0;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // --- timing --------------------------------------------------------
        const double now = glfwGetTime();
        float frame_dt = static_cast<float>(now - app.last_time);
        app.last_time = now;
        if (!(frame_dt > 0.0f)) {
            frame_dt = 1.0f / 60.0f;
        }
        if (frame_dt > 0.25f) {
            frame_dt = 0.25f;
        }
        app.frame_ms = frame_dt * 1000.0f;
        app.control.fps = 1.0f / frame_dt;

        // --- simulation ----------------------------------------------------
        Profiler& profiler = Profiler::global();
        profiler.begin_frame();

        if (app.control.reset_requested) {
            build_scene(app, app.seed);
            app.control.reset_requested = false;
            app.control.step_requested = false;
        }
        if (app.control.paused) {
            if (app.control.step_requested) {
                app.loop.step_once();
                app.control.step_requested = false;
            }
        } else {
            app.loop.advance(frame_dt);
        }

        profiler.add_sample("frame", app.frame_ms);
        profiler.end_frame();

        // --- live query throughput ----------------------------------------
        {
            const int query_count = 200;
            const double t0 = glfwGetTime();
            for (int i = 0; i < query_count; ++i) {
                const float x = -6.0f + 12.0f * (static_cast<float>(i) / query_count);
                app.physics.raycast(Vec3(x, 12.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), 100.0f);
            }
            const double elapsed = glfwGetTime() - t0;
            if (elapsed > 1.0e-6) {
                app.queries_per_sec = static_cast<float>(query_count / elapsed);
            }
            app.control.queries_per_sec = app.queries_per_sec;
        }

        // --- input ---------------------------------------------------------
        handle_navigation_input(app);

        // --- UI ------------------------------------------------------------
        const ImVec2 display_size = ImGui::GetIO().DisplaySize;
        const float ui_w = display_size.x > 1.0f ? display_size.x : 1.0f;
        const float ui_h = display_size.y > 1.0f ? display_size.y : 1.0f;

        // Re-run the layout math when the window changes size so the panels
        // follow the viewport (nothing here is hardcoded in pixels).
        if (std::fabs(ui_w - app.layout.viewport_width()) > 0.5f ||
            std::fabs(ui_h - app.layout.viewport_height()) > 0.5f) {
            app.layout.compute_layout(ui_w, ui_h);
            app.ui.set_items_from_layout(app.layout);
        }

        const ImGuiWindowFlags overlay_flags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
            ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus |
            ImGuiWindowFlags_NoSavedSettings;
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(ui_w, ui_h));
        if (ImGui::Begin("##viewport", nullptr, overlay_flags)) {
            if (app.control.wireframe) {
                ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(0, 0), ImVec2(ui_w, ui_h),
                                                          IM_COL32(0, 0, 0, 255));
            }
            draw_scene_lines(app, ImGui::GetWindowDrawList(), ui_w, ui_h);
        }
        ImGui::End();

        handle_mouse_selection(app, ui_w, ui_h);

        for (const Widget& widget : app.layout.widgets()) {
            draw_widget(app, widget);
        }
        draw_entity_list(app);
        draw_stats_overlay(app);

        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        ++frame_count;
        if (options.max_frames > 0 && frame_count >= options.max_frames) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    std::printf("minieng_sandbox: rendered %d frame(s), %zu entities, %zu widgets, %llu ticks\n",
                frame_count, app.world.entity_count(), app.layout.widget_count(),
                static_cast<unsigned long long>(app.loop.tick_count()));
    return 0;
}
