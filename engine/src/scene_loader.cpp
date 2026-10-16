#include "minieng/scene_loader.hpp"

#include <fstream>
#include <sstream>

#include "minieng/json.hpp"

namespace minieng {

namespace {

bool read_vec3(const Json& node, Vec3& out) {
    if (!node.is_array() || node.size() < 3) {
        return false;
    }
    out = Vec3(node.at(0).as_float(), node.at(1).as_float(), node.at(2).as_float());
    return true;
}

bool parse_shape(const std::string& name, Collider::Shape& out) {
    if (name.empty() || name == "aabb" || name == "box") {
        out = Collider::AABB;
        return true;
    }
    if (name == "obb") {
        out = Collider::OBB;
        return true;
    }
    if (name == "sphere" || name == "ball") {
        out = Collider::Sphere;
        return true;
    }
    return false;
}

} // namespace

SceneDescription load_scene_from_string(const std::string& text, World& world, Scene& scene) {
    SceneDescription desc;

    std::string error;
    const Json root = Json::parse(text, &error);
    if (!error.empty()) {
        desc.error = error;
        return desc;
    }
    if (!root.is_object()) {
        desc.error = "scene root must be a JSON object";
        return desc;
    }

    desc.name = root["name"].as_string();
    desc.seed = static_cast<std::uint64_t>(root["seed"].as_number(1337.0));
    read_vec3(root["gravity"], desc.gravity);

    if (root["camera"].is_object()) {
        const Json& camera = root["camera"];
        read_vec3(camera["eye"], desc.camera_eye);
        read_vec3(camera["target"], desc.camera_target);
    }

    const Json& entities = root["entities"];
    if (!entities.is_array() || entities.size() == 0) {
        desc.error = "scene must contain a non-empty 'entities' array";
        return desc;
    }

    for (const Json& node : entities.items()) {
        const std::string shape_name = node["shape"].as_string();
        Collider::Shape shape = Collider::AABB;
        if (!parse_shape(shape_name, shape)) {
            desc.error = "unknown shape: " + shape_name;
            return desc;
        }

        Vec3 position = Vec3::zero();
        Vec3 half_extents(0.5f, 0.5f, 0.5f);
        Vec3 gravity_offset = Vec3::zero();
        Vec3 euler_degrees = Vec3::zero();
        read_vec3(node["position"], position);
        read_vec3(node["half_extents"], half_extents);
        read_vec3(node["offset"], gravity_offset);
        read_vec3(node["rotation_euler_deg"], euler_degrees);

        const float radius = node["radius"].as_float(0.5f);
        const float mass = node["mass"].as_float(1.0f);
        const float restitution = node["restitution"].as_float(0.3f);
        const float friction = node["friction"].as_float(0.4f);
        const bool is_static = node["static"].as_bool(false);
        const std::string tag = node["tag"].as_string();
        const int repeat = node["repeat"].is_number() ? node["repeat"].as_int(1) : 1;

        if (repeat < 1) {
            desc.error = "entity 'repeat' must be at least 1";
            return desc;
        }

        for (int copy = 0; copy < repeat; ++copy) {
            Collider collider;
            collider.shape = shape;
            collider.half_extents = half_extents;
            collider.radius = radius;

            RigidBody body;
            body.is_static = is_static;
            body.mass = is_static ? 0.0f : mass;
            body.restitution = restitution;
            body.friction = friction;

            Transform transform;
            transform.position = position + gravity_offset * static_cast<float>(copy);
            transform.rotation =
                Quat::from_euler(deg_to_rad(euler_degrees.x), deg_to_rad(euler_degrees.y),
                                 deg_to_rad(euler_degrees.z));

            const EntityId id = world.create_entity();
            const int scene_node = scene.create_node(-1, id);
            scene.set_local(scene_node, transform);
            world.add_transform_ref(id, scene_node);
            world.add_collider(id, collider);
            world.add_rigid_body(id, body);
            if (!tag.empty()) {
                std::ostringstream name;
                name << tag;
                if (repeat > 1) {
                    name << '_' << copy;
                }
                world.add_tag(id, name.str());
            }
            desc.entities.push_back(id);
        }
    }

    scene.update_world_transforms();
    desc.ok = true;
    return desc;
}

SceneDescription load_scene_from_file(const std::string& path, World& world, Scene& scene) {
    std::ifstream file(path);
    if (!file) {
        SceneDescription desc;
        desc.error = "could not open scene file: " + path;
        return desc;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return load_scene_from_string(buffer.str(), world, scene);
}

} // namespace minieng
