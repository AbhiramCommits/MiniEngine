#pragma once

// Binding table: named getters/setters that connect UI controls to live
// simulation state. The UI layer stays agnostic about where a value lives; a
// slider bound to "physics.gravity.y" just reads and writes through a function.

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "minieng/physics.hpp"
#include "minieng/scene.hpp"
#include "minieng/types.hpp"
#include "minieng/world.hpp"

namespace minieng {
namespace ui {

class BindingTable {
public:
    using FloatGetter = std::function<float()>;
    using FloatSetter = std::function<void(float)>;
    using BoolGetter = std::function<bool()>;
    using BoolSetter = std::function<void(bool)>;
    using StringGetter = std::function<std::string()>;
    using Action = std::function<void()>;

    struct Entry {
        enum class Kind { Float, Bool, String, Action };

        Kind kind = Kind::Float;
        FloatGetter get_float;
        FloatSetter set_float;
        BoolGetter get_bool;
        BoolSetter set_bool;
        StringGetter get_string;
        Action action;
        float min = 0.0f;
        float max = 1.0f;
        bool read_only = false;
    };

    void bind_float(const std::string& name, FloatGetter getter, FloatSetter setter, float min,
                    float max);
    void bind_readonly_float(const std::string& name, FloatGetter getter, float min, float max);
    void bind_bool(const std::string& name, BoolGetter getter, BoolSetter setter);
    void bind_string(const std::string& name, StringGetter getter);
    void bind_action(const std::string& name, Action action);

    bool has(const std::string& name) const;
    const Entry* find_entry(const std::string& name) const;

    bool is_float(const std::string& name) const;
    bool is_bool(const std::string& name) const;
    bool is_string(const std::string& name) const;
    bool is_action(const std::string& name) const;
    bool is_read_only(const std::string& name) const;

    float get_float(const std::string& name, float fallback = 0.0f) const;
    void set_float(const std::string& name, float value);
    bool get_bool(const std::string& name, bool fallback = false) const;
    void set_bool(const std::string& name, bool value);
    std::string get_string(const std::string& name) const;

    // Runs an action binding; returns false when the name is not an action.
    bool activate(const std::string& name);
    bool toggle(const std::string& name);

    // Insertion order, which keeps menus and tests deterministic.
    const std::vector<std::string>& names() const { return names_; }

    void clear();

private:
    std::vector<std::pair<std::string, Entry>> entries_;
    std::vector<std::string> names_;
    std::unordered_map<std::string, std::size_t> index_;
};

// Simulation-side control flags that UI actions flip. The app (or a test) owns
// one of these and reacts to the flags.
struct SimControl {
    bool paused = false;
    bool step_requested = false;
    bool reset_requested = false;
    bool wireframe = false;

    float fps = 0.0f;
    std::uint64_t ticks = 0;
    int body_count = 0;
    int pair_count = 0;
    int contact_count = 0;
    float queries_per_sec = 0.0f;
};

// Registers the standard sandbox bindings over a world plus physics config.
// `selected` is read/written so inspector fields follow the current selection.
void register_simulation_bindings(BindingTable& table, World& world, Scene& scene,
                                  PhysicsConfig& config, EntityId& selected, SimControl& control);

} // namespace ui
} // namespace minieng
