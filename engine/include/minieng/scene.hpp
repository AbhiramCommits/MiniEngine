#pragma once

// Scene graph: a flat node pool with parent/child links and cached world
// transforms. Transforms compose top-down (`world = parent_world * local`),
// which is the standard row-major / column-vector convention used everywhere in
// MiniEngine.

#include <unordered_map>
#include <vector>

#include "minieng/math.hpp"
#include "minieng/types.hpp"

namespace minieng {

struct Transform {
    Vec3 position;
    Quat rotation = Quat::identity();
    Vec3 scale = Vec3::one();

    Mat4 to_matrix() const { return Mat4::from_trs(position, rotation, scale); }
};

struct SceneNode {
    EntityId id = kInvalidEntity; // owning entity (optional)
    int parent = -1;              // -1 == root
    std::vector<int> children;
    Transform local;
    Mat4 world = Mat4::identity();
};

class Scene {
public:
    // Creates a node under `parent` (-1 for a root) and returns its index.
    // `owner` optionally links the node to an entity for reverse lookups.
    int create_node(int parent = -1, EntityId owner = kInvalidEntity);

    bool valid(int node) const {
        return node >= 0 && static_cast<std::size_t>(node) < nodes_.size();
    }

    int node_count() const { return static_cast<int>(nodes_.size()); }
    const std::vector<SceneNode>& nodes() const { return nodes_; }
    const SceneNode& node(int i) const { return nodes_[static_cast<std::size_t>(i)]; }

    const Transform& local_of(int node) const;
    Transform& local_of(int node);
    void set_local(int node, const Transform& t);

    const Mat4& world_of(int node) const;
    Vec3 world_position_of(int node) const;

    int parent_of(int node) const;

    // Moves `node` under `new_parent`. When `keep_world` is true the node's world
    // transform is preserved by recomputing its local transform relative to the
    // new parent. Reparenting a node under its own descendant is rejected.
    void reparent(int node, int new_parent, bool keep_world = false);

    // Single top-down pass: roots resolve first, then descendants, so every
    // `world` is correct after one traversal.
    void update_world_transforms();

    EntityId entity_of(int node) const;
    void set_entity(int node, EntityId id);
    int node_of_entity(EntityId id) const;

    void clear();

private:
    bool is_descendant_of(int node, int possible_ancestor) const;

    std::vector<SceneNode> nodes_;
    std::unordered_map<EntityId, int> entity_to_node_;
};

} // namespace minieng
