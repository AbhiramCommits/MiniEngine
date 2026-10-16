#pragma once

// Entity/component world.
//
// Storage is "dense arrays + id -> index map": every component type keeps one
// entry per live entity, aligned with the entity id array. Entities are always
// stored in ascending id order (ids are never recycled and destruction keeps
// order), so iterating `entities()` is a deterministic, stable order.
//
// Transform data itself lives in the Scene; a `TransformRef` is just a handle
// to the node that owns it.

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "minieng/math.hpp"
#include "minieng/scene.hpp"
#include "minieng/types.hpp"

namespace minieng {

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

struct TransformRef {
    int scene_node = -1;
};

struct RigidBody {
    Vec3 velocity;
    Vec3 angular_velocity;
    float mass = 1.0f;
    float restitution = 0.3f;
    float friction = 0.4f;
    bool is_static = false;

    float inverse_mass() const {
        if (is_static || mass <= 0.0f) {
            return 0.0f;  // infinite mass
        }
        return 1.0f / mass;
    }
};

struct Collider {
    enum Shape { AABB = 0, OBB = 1, Sphere = 2 };

    Shape shape = AABB;
    Vec3 half_extents = Vec3(0.5f, 0.5f, 0.5f);
    float radius = 0.5f;
};

struct Tag {
    std::string name;
};

// ---------------------------------------------------------------------------
// Snapshot / hashing
// ---------------------------------------------------------------------------

// A quantized copy of the simulation state plus an FNV-1a hash. Two runs are
// "identical" when their hashes match; values are rounded to 1e-4 first so that
// only meaningful drift counts as a difference.
struct WorldSnapshot {
    struct Entry {
        EntityId id = kInvalidEntity;
        Vec3 position;
        Quat rotation = Quat::identity();
        Vec3 velocity;
        Vec3 angular_velocity;
        bool has_transform = false;
        bool has_body = false;
    };

    std::vector<Entry> entries;

    std::uint64_t hash() const;
};

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

class World {
public:
    EntityId create_entity();
    void destroy_entity(EntityId id);

    bool alive(EntityId id) const;
    std::size_t entity_count() const { return ids_.size(); }
    const std::vector<EntityId>& entities() const { return ids_; }

    // Position of `id` in the dense arrays, or -1 when it is not alive.
    int index_of(EntityId id) const;
    EntityId next_id() const { return next_id_; }

    // --- component access -------------------------------------------------
    TransformRef& add_transform_ref(EntityId id, int scene_node = -1);
    TransformRef* get_transform_ref(EntityId id);
    const TransformRef* get_transform_ref(EntityId id) const;

    RigidBody& add_rigid_body(EntityId id, const RigidBody& body = RigidBody{});
    RigidBody* get_rigid_body(EntityId id);
    const RigidBody* get_rigid_body(EntityId id) const;

    Collider& add_collider(EntityId id, const Collider& collider = Collider{});
    Collider* get_collider(EntityId id);
    const Collider* get_collider(EntityId id) const;

    Tag& add_tag(EntityId id, const std::string& name = std::string());
    Tag* get_tag(EntityId id);
    const Tag* get_tag(EntityId id) const;

    bool has_transform_ref(EntityId id) const { return get_transform_ref(id) != nullptr; }
    bool has_rigid_body(EntityId id) const { return get_rigid_body(id) != nullptr; }
    bool has_collider(EntityId id) const { return get_collider(id) != nullptr; }
    bool has_tag(EntityId id) const { return get_tag(id) != nullptr; }

    // --- queries ----------------------------------------------------------
    std::vector<EntityId> entities_with_rigid_body() const;
    std::vector<EntityId> entities_with_collider() const;

    // Reads transforms (via TransformRef -> Scene) and velocities into a
    // quantized snapshot. Entities are visited in ascending id order.
    WorldSnapshot snapshot(const Scene& scene) const;
    std::uint64_t hash(const Scene& scene) const { return snapshot(scene).hash(); }

    void clear();

private:
    template <typename T>
    struct ComponentArray {
        std::vector<T> items;
        std::vector<std::uint8_t> present;
        // Returned by add() when asked for a dead entity; keeps the API
        // reference-based without relying on a dangling temporary.
        T fallback{};
    };

    template <typename T>
    T* get(ComponentArray<T>& store, EntityId id) {
        const int idx = index_of(id);
        if (idx < 0 || store.present[static_cast<std::size_t>(idx)] == 0) {
            return nullptr;
        }
        return &store.items[static_cast<std::size_t>(idx)];
    }

    template <typename T>
    const T* get(const ComponentArray<T>& store, EntityId id) const {
        const int idx = index_of(id);
        if (idx < 0 || store.present[static_cast<std::size_t>(idx)] == 0) {
            return nullptr;
        }
        return &store.items[static_cast<std::size_t>(idx)];
    }

    template <typename T>
    T& add(ComponentArray<T>& store, EntityId id, const T& value) {
        const int idx = index_of(id);
        // Callers pass live entities; guard defensively anyway.
        if (idx < 0) {
            store.fallback = value;
            return store.fallback;
        }
        store.items[static_cast<std::size_t>(idx)] = value;
        store.present[static_cast<std::size_t>(idx)] = 1;
        return store.items[static_cast<std::size_t>(idx)];
    }

    void grow_components();
    void erase_components_at(int idx);

    std::vector<EntityId> ids_;
    std::unordered_map<EntityId, int> id_to_index_;
    EntityId next_id_ = 1;

    ComponentArray<TransformRef> transform_refs_;
    ComponentArray<RigidBody> rigid_bodies_;
    ComponentArray<Collider> colliders_;
    ComponentArray<Tag> tags_;
};

}  // namespace minieng
