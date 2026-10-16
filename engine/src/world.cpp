#include "minieng/world.hpp"

#include <algorithm>
#include <cmath>

namespace minieng {

namespace {

// FNV-1a constants.
constexpr std::uint64_t kFnvOffset = 14695981039346656037ull;
constexpr std::uint64_t kFnvPrime = 1099511628211ull;

inline std::uint64_t fnv_step(std::uint64_t h, const void* data, std::size_t len) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < len; ++i) {
        h ^= static_cast<std::uint64_t>(bytes[i]);
        h *= kFnvPrime;
    }
    return h;
}

template <typename T>
inline std::uint64_t fnv_scalar(std::uint64_t h, const T& value) {
    return fnv_step(h, &value, sizeof(T));
}

// Quantize to 1e-4 so only drift that matters changes the hash.
inline std::int64_t quantize(float v) {
    return static_cast<std::int64_t>(std::llround(static_cast<double>(v) * 10000.0));
}

}  // namespace

// ---------------------------------------------------------------------------
// World lifecycle
// ---------------------------------------------------------------------------

EntityId World::create_entity() {
    const EntityId id = next_id_++;
    ids_.push_back(id);
    id_to_index_[id] = static_cast<int>(ids_.size()) - 1;
    grow_components();
    return id;
}

void World::destroy_entity(EntityId id) {
    const int idx = index_of(id);
    if (idx < 0) {
        return;
    }
    ids_.erase(ids_.begin() + idx);
    erase_components_at(idx);
    // ids_ keeps ascending order, so only the tail indices shift.
    for (std::size_t i = static_cast<std::size_t>(idx); i < ids_.size(); ++i) {
        id_to_index_[ids_[i]] = static_cast<int>(i);
    }
    id_to_index_.erase(id);
}

void World::grow_components() {
    transform_refs_.items.emplace_back();
    transform_refs_.present.push_back(0);
    rigid_bodies_.items.emplace_back();
    rigid_bodies_.present.push_back(0);
    colliders_.items.emplace_back();
    colliders_.present.push_back(0);
    tags_.items.emplace_back();
    tags_.present.push_back(0);
}

void World::erase_components_at(int idx) {
    const auto erase = [idx](auto& store) {
        store.items.erase(store.items.begin() + idx);
        store.present.erase(store.present.begin() + idx);
    };
    erase(transform_refs_);
    erase(rigid_bodies_);
    erase(colliders_);
    erase(tags_);
}

bool World::alive(EntityId id) const { return index_of(id) >= 0; }

int World::index_of(EntityId id) const {
    const auto it = id_to_index_.find(id);
    return it == id_to_index_.end() ? -1 : it->second;
}

void World::clear() {
    ids_.clear();
    id_to_index_.clear();
    next_id_ = 1;
    transform_refs_ = ComponentArray<TransformRef>{};
    rigid_bodies_ = ComponentArray<RigidBody>{};
    colliders_ = ComponentArray<Collider>{};
    tags_ = ComponentArray<Tag>{};
}

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

TransformRef& World::add_transform_ref(EntityId id, int scene_node) {
    TransformRef ref;
    ref.scene_node = scene_node;
    return add(transform_refs_, id, ref);
}

TransformRef* World::get_transform_ref(EntityId id) { return get(transform_refs_, id); }
const TransformRef* World::get_transform_ref(EntityId id) const { return get(transform_refs_, id); }

RigidBody& World::add_rigid_body(EntityId id, const RigidBody& body) {
    return add(rigid_bodies_, id, body);
}

RigidBody* World::get_rigid_body(EntityId id) { return get(rigid_bodies_, id); }
const RigidBody* World::get_rigid_body(EntityId id) const { return get(rigid_bodies_, id); }

Collider& World::add_collider(EntityId id, const Collider& collider) {
    return add(colliders_, id, collider);
}

Collider* World::get_collider(EntityId id) { return get(colliders_, id); }
const Collider* World::get_collider(EntityId id) const { return get(colliders_, id); }

Tag& World::add_tag(EntityId id, const std::string& name) {
    Tag t;
    t.name = name;
    return add(tags_, id, t);
}

Tag* World::get_tag(EntityId id) { return get(tags_, id); }
const Tag* World::get_tag(EntityId id) const { return get(tags_, id); }

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

std::vector<EntityId> World::entities_with_rigid_body() const {
    std::vector<EntityId> out;
    for (const EntityId id : ids_) {
        if (get(rigid_bodies_, id) != nullptr) {
            out.push_back(id);
        }
    }
    return out;
}

std::vector<EntityId> World::entities_with_collider() const {
    std::vector<EntityId> out;
    for (const EntityId id : ids_) {
        if (get(colliders_, id) != nullptr) {
            out.push_back(id);
        }
    }
    return out;
}

WorldSnapshot World::snapshot(const Scene& scene) const {
    WorldSnapshot snap;
    snap.entries.reserve(ids_.size());

    for (const EntityId id : ids_) {
        WorldSnapshot::Entry entry;
        entry.id = id;

        if (const TransformRef* ref = get(transform_refs_, id)) {
            if (ref->scene_node >= 0 && scene.valid(ref->scene_node)) {
                entry.position = scene.world_position_of(ref->scene_node);
                entry.rotation = scene.local_of(ref->scene_node).rotation;
                entry.has_transform = true;
            }
        }
        if (const RigidBody* body = get(rigid_bodies_, id)) {
            entry.velocity = body->velocity;
            entry.angular_velocity = body->angular_velocity;
            entry.has_body = true;
        }
        snap.entries.push_back(entry);
    }
    return snap;
}

std::uint64_t WorldSnapshot::hash() const {
    std::uint64_t h = kFnvOffset;
    for (const Entry& e : entries) {
        h = fnv_scalar(h, e.id);
        h = fnv_scalar(h, quantize(e.position.x));
        h = fnv_scalar(h, quantize(e.position.y));
        h = fnv_scalar(h, quantize(e.position.z));
        h = fnv_scalar(h, quantize(e.rotation.w));
        h = fnv_scalar(h, quantize(e.rotation.x));
        h = fnv_scalar(h, quantize(e.rotation.y));
        h = fnv_scalar(h, quantize(e.rotation.z));
        h = fnv_scalar(h, quantize(e.velocity.x));
        h = fnv_scalar(h, quantize(e.velocity.y));
        h = fnv_scalar(h, quantize(e.velocity.z));
        h = fnv_scalar(h, quantize(e.angular_velocity.x));
        h = fnv_scalar(h, quantize(e.angular_velocity.y));
        h = fnv_scalar(h, quantize(e.angular_velocity.z));
    }
    return h;
}

}  // namespace minieng
