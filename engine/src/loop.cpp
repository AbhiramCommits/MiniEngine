#include "minieng/loop.hpp"

#include <algorithm>

namespace minieng {

const char* system_stage_name(SystemStage stage) {
    switch (stage) {
        case SystemStage::Input: return "input";
        case SystemStage::Spawn: return "spawn";
        case SystemStage::PhysicsIntegrate: return "integrate";
        case SystemStage::CollisionResolve: return "collision_resolve";
        case SystemStage::SceneTransform: return "scene_transform";
        case SystemStage::UiObserve: return "ui_observe";
        case SystemStage::Count: break;
    }
    return "unknown";
}

FixedLoop::FixedLoop(float dt, int max_substeps) {
    set_dt(dt);
    set_max_substeps(max_substeps);
}

void FixedLoop::set_dt(float dt) {
    if (dt > 0.0f && dt == dt) {  // reject NaN / non-positive
        dt_ = dt;
    }
}

void FixedLoop::set_max_substeps(int n) {
    if (n > 0) {
        max_substeps_ = n;
    }
}

void FixedLoop::set_system(SystemStage stage, System fn) {
    const int index = static_cast<int>(stage);
    if (index < 0 || index >= kSystemStageCount) {
        return;
    }
    systems_[static_cast<std::size_t>(index)] = std::move(fn);
}

void FixedLoop::clear_systems() {
    for (auto& s : systems_) {
        s = nullptr;
    }
}

bool FixedLoop::has_system(SystemStage stage) const {
    const int index = static_cast<int>(stage);
    return index >= 0 && index < kSystemStageCount &&
           static_cast<bool>(systems_[static_cast<std::size_t>(index)]);
}

void FixedLoop::step_once() {
    for (const auto& system : systems_) {
        if (system) {
            system(dt_);
        }
    }
    ++tick_count_;
}

int FixedLoop::advance(float frame_dt) {
    // Guard against NaN / negative deltas from a paused or first frame.
    if (!(frame_dt > 0.0f)) {
        return 0;
    }
    accumulator_ += frame_dt;

    int steps = 0;
    while (accumulator_ >= dt_ && steps < max_substeps_) {
        step_once();
        accumulator_ -= dt_;
        ++steps;
    }

    // If we saturated the substep budget there is a backlog. Keep at most one
    // pending step so a slow frame cannot cascade forever.
    if (steps == max_substeps_ && accumulator_ > dt_) {
        accumulator_ = dt_;
    }
    return steps;
}

void FixedLoop::reset() {
    accumulator_ = 0.0f;
    tick_count_ = 0;
}

}  // namespace minieng
