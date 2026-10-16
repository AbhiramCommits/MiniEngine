#pragma once

// Fixed-timestep loop.
//
// Simulation runs on a fixed dt (default 1/60 s) driven by an accumulator, with
// a hard cap on how many substeps a single frame may consume so a slow frame
// cannot spiral into an unbounded catch-up. Systems are registered per stage and
// always execute in the order listed by SystemStage.

#include <array>
#include <cstdint>
#include <functional>

namespace minieng {

// Explicit, ordered simulation stages. Never reorder without updating every
// consumer that relies on the sequence.
enum class SystemStage : int {
    Input = 0,        // sample devices / replay input
    Spawn,            // scripts, spawners, gameplay logic
    PhysicsIntegrate, // semi-implicit Euler, damping, angular integration
    CollisionResolve, // broadphase -> narrowphase -> impulses
    SceneTransform,   // recompute world matrices from local transforms
    UiObserve,        // immediate-mode UI, observers, profiler readout
    Count
};

constexpr int kSystemStageCount = static_cast<int>(SystemStage::Count);

const char* system_stage_name(SystemStage stage);

class FixedLoop {
public:
    using System = std::function<void(float dt)>;

    explicit FixedLoop(float dt = 1.0f / 60.0f, int max_substeps = 8);

    void set_system(SystemStage stage, System fn);
    void clear_systems();
    bool has_system(SystemStage stage) const;

    float dt() const { return dt_; }
    void set_dt(float dt);

    int max_substeps() const { return max_substeps_; }
    void set_max_substeps(int n);

    std::uint64_t tick_count() const { return tick_count_; }
    void set_tick_count(std::uint64_t ticks) { tick_count_ = ticks; }

    float accumulator() const { return accumulator_; }

    // Fraction of the way to the next fixed step; useful for render
    // interpolation. Always in [0, 1).
    float alpha() const { return dt_ > 0.0f ? accumulator_ / dt_ : 0.0f; }

    // Runs exactly one fixed step through all registered stages.
    void step_once();

    // Consumes `frame_dt` seconds of wall time and runs as many fixed steps as
    // it contains, capped at max_substeps. Returns the number of steps run.
    int advance(float frame_dt);

    void reset();

private:
    float dt_ = 1.0f / 60.0f;
    float accumulator_ = 0.0f;
    int max_substeps_ = 8;
    std::uint64_t tick_count_ = 0;
    std::array<System, kSystemStageCount> systems_{};
};

} // namespace minieng
