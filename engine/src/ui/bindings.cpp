#include "minieng/ui/bindings.hpp"

#include <sstream>

namespace minieng {
namespace ui {

void BindingTable::bind_float(const std::string& name, FloatGetter getter, FloatSetter setter,
                              float min, float max) {
    Entry entry;
    entry.kind = Entry::Kind::Float;
    entry.get_float = std::move(getter);
    entry.set_float = std::move(setter);
    entry.min = min;
    entry.max = max;
    entry.read_only = false;
    index_[name] = entries_.size();
    entries_.emplace_back(name, std::move(entry));
    names_.push_back(name);
}

void BindingTable::bind_readonly_float(const std::string& name, FloatGetter getter, float min,
                                       float max) {
    bind_float(name, std::move(getter), nullptr, min, max);
    entries_.back().second.read_only = true;
}

void BindingTable::bind_bool(const std::string& name, BoolGetter getter, BoolSetter setter) {
    Entry entry;
    entry.kind = Entry::Kind::Bool;
    entry.get_bool = std::move(getter);
    entry.set_bool = std::move(setter);
    index_[name] = entries_.size();
    entries_.emplace_back(name, std::move(entry));
    names_.push_back(name);
}

void BindingTable::bind_string(const std::string& name, StringGetter getter) {
    Entry entry;
    entry.kind = Entry::Kind::String;
    entry.get_string = std::move(getter);
    entry.read_only = true;
    index_[name] = entries_.size();
    entries_.emplace_back(name, std::move(entry));
    names_.push_back(name);
}

void BindingTable::bind_action(const std::string& name, Action action) {
    Entry entry;
    entry.kind = Entry::Kind::Action;
    entry.action = std::move(action);
    index_[name] = entries_.size();
    entries_.emplace_back(name, std::move(entry));
    names_.push_back(name);
}

bool BindingTable::has(const std::string& name) const { return index_.count(name) > 0; }

const BindingTable::Entry* BindingTable::find_entry(const std::string& name) const {
    const auto it = index_.find(name);
    return it == index_.end() ? nullptr : &entries_[it->second].second;
}

bool BindingTable::is_float(const std::string& name) const {
    const Entry* e = find_entry(name);
    return e != nullptr && e->kind == Entry::Kind::Float;
}

bool BindingTable::is_bool(const std::string& name) const {
    const Entry* e = find_entry(name);
    return e != nullptr && e->kind == Entry::Kind::Bool;
}

bool BindingTable::is_string(const std::string& name) const {
    const Entry* e = find_entry(name);
    return e != nullptr && e->kind == Entry::Kind::String;
}

bool BindingTable::is_action(const std::string& name) const {
    const Entry* e = find_entry(name);
    return e != nullptr && e->kind == Entry::Kind::Action;
}

bool BindingTable::is_read_only(const std::string& name) const {
    const Entry* e = find_entry(name);
    return e == nullptr || e->read_only;
}

float BindingTable::get_float(const std::string& name, float fallback) const {
    const Entry* e = find_entry(name);
    if (e == nullptr || e->kind != Entry::Kind::Float || !e->get_float) {
        return fallback;
    }
    return e->get_float();
}

void BindingTable::set_float(const std::string& name, float value) {
    const Entry* e = find_entry(name);
    if (e == nullptr || e->kind != Entry::Kind::Float || !e->set_float || e->read_only) {
        return;
    }
    e->set_float(clampf(value, e->min, e->max));
}

bool BindingTable::get_bool(const std::string& name, bool fallback) const {
    const Entry* e = find_entry(name);
    if (e == nullptr || e->kind != Entry::Kind::Bool || !e->get_bool) {
        return fallback;
    }
    return e->get_bool();
}

void BindingTable::set_bool(const std::string& name, bool value) {
    const Entry* e = find_entry(name);
    if (e == nullptr || e->kind != Entry::Kind::Bool || !e->set_bool) {
        return;
    }
    e->set_bool(value);
}

std::string BindingTable::get_string(const std::string& name) const {
    const Entry* e = find_entry(name);
    if (e == nullptr) {
        return std::string();
    }
    if (e->kind == Entry::Kind::String && e->get_string) {
        return e->get_string();
    }
    if (e->kind == Entry::Kind::Float && e->get_float) {
        std::ostringstream oss;
        oss.precision(3);
        oss << std::fixed << e->get_float();
        return oss.str();
    }
    if (e->kind == Entry::Kind::Bool && e->get_bool) {
        return e->get_bool() ? "true" : "false";
    }
    return std::string();
}

bool BindingTable::activate(const std::string& name) {
    const Entry* e = find_entry(name);
    if (e == nullptr) {
        return false;
    }
    if (e->kind == Entry::Kind::Action && e->action) {
        e->action();
        return true;
    }
    if (e->kind == Entry::Kind::Bool) {
        return toggle(name);
    }
    return false;
}

bool BindingTable::toggle(const std::string& name) {
    const Entry* e = find_entry(name);
    if (e == nullptr || e->kind != Entry::Kind::Bool || !e->get_bool || !e->set_bool) {
        return false;
    }
    e->set_bool(!e->get_bool());
    return true;
}

void BindingTable::clear() {
    entries_.clear();
    names_.clear();
    index_.clear();
}

// ---------------------------------------------------------------------------
// Simulation bindings
// ---------------------------------------------------------------------------

namespace {

Transform* selected_transform(World& world, Scene& scene, EntityId selected) {
    TransformRef* ref = world.get_transform_ref(selected);
    if (ref == nullptr || ref->scene_node < 0 || !scene.valid(ref->scene_node)) {
        return nullptr;
    }
    return &scene.local_of(ref->scene_node);
}

RigidBody* selected_body(World& world, EntityId selected) {
    return world.get_rigid_body(selected);
}

}  // namespace

void register_simulation_bindings(BindingTable& table, World& world, Scene& scene,
                                  PhysicsConfig& config, EntityId& selected, SimControl& control) {
    // --- physics tuning ---------------------------------------------------
    table.bind_float(
        "physics.gravity.y", [&config]() { return config.gravity.y; },
        [&config](float v) { config.gravity.y = v; }, -30.0f, 10.0f);
    table.bind_float(
        "physics.gravity.x", [&config]() { return config.gravity.x; },
        [&config](float v) { config.gravity.x = v; }, -20.0f, 20.0f);
    table.bind_float(
        "physics.restitution",
        [&config, &world, &selected]() {
            // Show the selected body's value when there is one.
            if (const RigidBody* b = world.get_rigid_body(selected)) {
                return b->restitution;
            }
            return config.default_restitution;
        },
        [&config, &world, &selected](float v) {
            if (RigidBody* b = world.get_rigid_body(selected)) {
                b->restitution = v;
            } else {
                config.default_restitution = v;
            }
        },
        0.0f, 1.0f);
    table.bind_float(
        "physics.friction",
        [&config, &world, &selected]() {
            if (const RigidBody* b = world.get_rigid_body(selected)) {
                return b->friction;
            }
            return config.default_friction;
        },
        [&config, &world, &selected](float v) {
            if (RigidBody* b = world.get_rigid_body(selected)) {
                b->friction = v;
            } else {
                config.default_friction = v;
            }
        },
        0.0f, 1.0f);
    table.bind_float(
        "physics.damping", [&config]() { return config.linear_damping; },
        [&config](float v) { config.linear_damping = v; }, 0.0f, 1.0f);
    table.bind_float(
        "physics.angular_damping", [&config]() { return config.angular_damping; },
        [&config](float v) { config.angular_damping = v; }, 0.0f, 1.0f);

    // --- selected entity --------------------------------------------------
    table.bind_float(
        "selected.position.x",
        [&world, &scene, &selected]() {
            Transform* t = selected_transform(world, scene, selected);
            return t ? t->position.x : 0.0f;
        },
        [&world, &scene, &selected](float v) {
            if (Transform* t = selected_transform(world, scene, selected)) {
                t->position.x = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.position.y",
        [&world, &scene, &selected]() {
            Transform* t = selected_transform(world, scene, selected);
            return t ? t->position.y : 0.0f;
        },
        [&world, &scene, &selected](float v) {
            if (Transform* t = selected_transform(world, scene, selected)) {
                t->position.y = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.position.z",
        [&world, &scene, &selected]() {
            Transform* t = selected_transform(world, scene, selected);
            return t ? t->position.z : 0.0f;
        },
        [&world, &scene, &selected](float v) {
            if (Transform* t = selected_transform(world, scene, selected)) {
                t->position.z = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.velocity.x",
        [&world, &selected]() {
            RigidBody* b = selected_body(world, selected);
            return b ? b->velocity.x : 0.0f;
        },
        [&world, &selected](float v) {
            if (RigidBody* b = selected_body(world, selected)) {
                b->velocity.x = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.velocity.y",
        [&world, &selected]() {
            RigidBody* b = selected_body(world, selected);
            return b ? b->velocity.y : 0.0f;
        },
        [&world, &selected](float v) {
            if (RigidBody* b = selected_body(world, selected)) {
                b->velocity.y = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.velocity.z",
        [&world, &selected]() {
            RigidBody* b = selected_body(world, selected);
            return b ? b->velocity.z : 0.0f;
        },
        [&world, &selected](float v) {
            if (RigidBody* b = selected_body(world, selected)) {
                b->velocity.z = v;
            }
        },
        -50.0f, 50.0f);
    table.bind_float(
        "selected.mass",
        [&world, &selected]() {
            const RigidBody* b = world.get_rigid_body(selected);
            return b ? b->mass : 0.0f;
        },
        [&world, &selected](float v) {
            if (RigidBody* b = world.get_rigid_body(selected)) {
                b->mass = v;
            }
        },
        0.1f, 100.0f);

    // --- simulation control ----------------------------------------------
    table.bind_bool(
        "sim.pause", [&control]() { return control.paused; },
        [&control](bool v) { control.paused = v; });
    table.bind_action("sim.step", [&control]() { control.step_requested = true; });
    table.bind_action("sim.reset", [&control]() { control.reset_requested = true; });
    table.bind_bool(
        "debug.wireframe", [&control]() { return control.wireframe; },
        [&control](bool v) { control.wireframe = v; });

    // --- live stats (read-only) ------------------------------------------
    table.bind_readonly_float("stats.fps", [&control]() { return control.fps; }, 0.0f, 1000.0f);
    table.bind_readonly_float(
        "stats.ticks", [&control]() { return static_cast<float>(control.ticks); }, 0.0f, 1.0e9f);
    table.bind_readonly_float("stats.bodies",
                              [&control]() { return static_cast<float>(control.body_count); },
                              0.0f, 1.0e9f);
    table.bind_readonly_float("stats.pairs",
                              [&control]() { return static_cast<float>(control.pair_count); },
                              0.0f, 1.0e9f);
    table.bind_readonly_float(
        "stats.queries_per_sec", [&control]() { return control.queries_per_sec; }, 0.0f, 1.0e12f);
}

}  // namespace ui
}  // namespace minieng
