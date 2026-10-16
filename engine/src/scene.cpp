#include "minieng/scene.hpp"

#include <utility>

namespace minieng {

namespace {
const Mat4 kIdentity = Mat4::identity();
}  // namespace

int Scene::create_node(int parent, EntityId owner) {
    SceneNode node;
    node.id = owner;
    node.parent = -1;

    const int index = static_cast<int>(nodes_.size());
    nodes_.push_back(std::move(node));

    if (parent >= 0 && valid(parent)) {
        nodes_[static_cast<std::size_t>(index)].parent = parent;
        nodes_[static_cast<std::size_t>(parent)].children.push_back(index);
    }
    if (owner != kInvalidEntity) {
        entity_to_node_[owner] = index;
    }
    return index;
}

const Transform& Scene::local_of(int node) const {
    return nodes_[static_cast<std::size_t>(node)].local;
}

Transform& Scene::local_of(int node) {
    return nodes_[static_cast<std::size_t>(node)].local;
}

void Scene::set_local(int node, const Transform& t) {
    if (!valid(node)) {
        return;
    }
    nodes_[static_cast<std::size_t>(node)].local = t;
}

const Mat4& Scene::world_of(int node) const {
    if (!valid(node)) {
        return kIdentity;
    }
    return nodes_[static_cast<std::size_t>(node)].world;
}

Vec3 Scene::world_position_of(int node) const {
    const Mat4& m = world_of(node);
    return Vec3(m.at(0, 3), m.at(1, 3), m.at(2, 3));
}

int Scene::parent_of(int node) const {
    return valid(node) ? nodes_[static_cast<std::size_t>(node)].parent : -1;
}

bool Scene::is_descendant_of(int node, int possible_ancestor) const {
    int cur = node;
    while (cur >= 0 && valid(cur)) {
        if (cur == possible_ancestor) {
            return true;
        }
        cur = nodes_[static_cast<std::size_t>(cur)].parent;
    }
    return false;
}

void Scene::reparent(int node, int new_parent, bool keep_world) {
    if (!valid(node)) {
        return;
    }
    if (new_parent == node || (new_parent >= 0 && !valid(new_parent))) {
        return;
    }
    // Reject cycles: the new parent must not live inside this node's subtree.
    if (new_parent >= 0 && is_descendant_of(new_parent, node)) {
        return;
    }

    // Capture the current world transform before the links change.
    const Mat4 old_world = world_of(node);

    const int old_parent = nodes_[static_cast<std::size_t>(node)].parent;
    if (old_parent >= 0) {
        auto& siblings = nodes_[static_cast<std::size_t>(old_parent)].children;
        for (std::size_t i = 0; i < siblings.size(); ++i) {
            if (siblings[i] == node) {
                siblings.erase(siblings.begin() + static_cast<std::ptrdiff_t>(i));
                break;
            }
        }
    }

    nodes_[static_cast<std::size_t>(node)].parent = new_parent;
    if (new_parent >= 0) {
        nodes_[static_cast<std::size_t>(new_parent)].children.push_back(node);
    }

    if (keep_world) {
        // local = inverse(new_parent_world) * old_world
        Mat4 rel = old_world;
        if (new_parent >= 0) {
            rel = nodes_[static_cast<std::size_t>(new_parent)].world.inverse_affine() * old_world;
        }
        Transform t;
        Mat4::decompose(rel, t.position, t.rotation, t.scale);
        nodes_[static_cast<std::size_t>(node)].local = t;
    }

    // Keep the cache consistent for immediate world_of() queries.
    update_world_transforms();
}

void Scene::update_world_transforms() {
    // Explicit stack instead of recursion: parents are always resolved before
    // their children, in a single pass over the tree.
    struct Pending {
        int index;
        const Mat4* parent_world;
    };

    std::vector<Pending> stack;
    stack.reserve(nodes_.size());
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].parent < 0) {
            stack.push_back({static_cast<int>(i), nullptr});
        }
    }

    while (!stack.empty()) {
        const Pending p = stack.back();
        stack.pop_back();

        SceneNode& n = nodes_[static_cast<std::size_t>(p.index)];
        const Mat4 local_matrix = n.local.to_matrix();
        n.world = (p.parent_world != nullptr) ? (*p.parent_world * local_matrix) : local_matrix;

        for (int child : n.children) {
            // nodes_ does not reallocate here, so this pointer stays valid.
            stack.push_back({child, &n.world});
        }
    }
}

EntityId Scene::entity_of(int node) const {
    return valid(node) ? nodes_[static_cast<std::size_t>(node)].id : kInvalidEntity;
}

void Scene::set_entity(int node, EntityId id) {
    if (!valid(node)) {
        return;
    }
    const EntityId previous = nodes_[static_cast<std::size_t>(node)].id;
    if (previous != kInvalidEntity) {
        entity_to_node_.erase(previous);
    }
    nodes_[static_cast<std::size_t>(node)].id = id;
    if (id != kInvalidEntity) {
        entity_to_node_[id] = node;
    }
}

int Scene::node_of_entity(EntityId id) const {
    const auto it = entity_to_node_.find(id);
    return it == entity_to_node_.end() ? -1 : it->second;
}

void Scene::clear() {
    nodes_.clear();
    entity_to_node_.clear();
}

}  // namespace minieng
