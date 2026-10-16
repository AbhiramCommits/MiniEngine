#pragma once

// Scene file loading.
//
// A scene file describes the initial contents of a World/Scene plus the physics
// and camera settings that go with it. The GLFW app and the benchmark both load
// the same files, and it is plain JSON so scenes can be authored by hand.

#include <cstdint>
#include <string>
#include <vector>

#include "minieng/math.hpp"
#include "minieng/scene.hpp"
#include "minieng/types.hpp"
#include "minieng/world.hpp"

namespace minieng {

struct SceneDescription {
    bool ok = false;
    std::string error;

    std::string name;
    std::uint64_t seed = 0;
    Vec3 gravity = Vec3(0.0f, -9.81f, 0.0f);
    Vec3 camera_eye = Vec3(0.0f, 10.0f, 18.0f);
    Vec3 camera_target = Vec3(0.0f, 1.5f, 0.0f);

    // Entities created, in creation order.
    std::vector<EntityId> entities;
};

// Parses `text` and populates `world` / `scene`. On failure `ok` is false, the
// error is set, and nothing is added.
SceneDescription load_scene_from_string(const std::string& text, World& world, Scene& scene);

// Same, reading from a file. A missing/unreadable file reports an error.
SceneDescription load_scene_from_file(const std::string& path, World& world, Scene& scene);

} // namespace minieng
