#pragma once

// Rigid-body physics: integration, broadphase, narrowphase, resolution, and the
// query API (raycast / sweep / overlap).
//
// Pipeline (matches the FixedLoop stage order):
//     integrate -> broadphase(cached world AABBs) -> narrowphase -> resolve
//
// Everything operates on the ECS World + Scene; there is no rendering or window
// dependency. Physics-driven bodies are expected to be scene roots, i.e. their
// local transform is their world transform (see README "Limitations").

#include <cstddef>
#include <vector>

#include "minieng/math.hpp"
#include "minieng/scene.hpp"
#include "minieng/types.hpp"
#include "minieng/world.hpp"

namespace minieng {

// ---------------------------------------------------------------------------
// Geometry primitives
// ---------------------------------------------------------------------------

struct Aabb {
    Vec3 min = Vec3(1.0e30f, 1.0e30f, 1.0e30f);
    Vec3 max = Vec3(-1.0e30f, -1.0e30f, -1.0e30f);

    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return (max - min) * 0.5f; }
    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void expand(const Vec3& p);
    void expand(const Aabb& other);
    Aabb expanded(const Vec3& e) const;

    // Inclusive overlap on all three axes.
    bool overlaps(const Aabb& o) const {
        return min.x <= o.max.x && o.min.x <= max.x && min.y <= o.max.y &&
               o.min.y <= max.y && min.z <= o.max.z && o.min.z <= max.z;
    }
};

// Oriented bounding box: center + three orthonormal world axes + half extents.
struct Obb {
    Vec3 center;
    Vec3 half_extents = Vec3(0.5f, 0.5f, 0.5f);
    Vec3 axis[3] = {Vec3::unit_x(), Vec3::unit_y(), Vec3::unit_z()};
};

// ---------------------------------------------------------------------------
// Pairs / contacts / query results
// ---------------------------------------------------------------------------

struct BroadphaseBody {
    EntityId id = kInvalidEntity;
    Aabb aabb;
};

struct Pair {
    EntityId a = kInvalidEntity;
    EntityId b = kInvalidEntity;
};

struct Contact {
    EntityId a = kInvalidEntity;
    EntityId b = kInvalidEntity;
    Vec3 normal;               // points from a to b
    float penetration = 0.0f;
    Vec3 point;                // approximate contact point (world)
};

struct RayHit {
    bool hit = false;
    EntityId entity = kInvalidEntity;
    float t = 0.0f;            // world distance along the (normalized) direction
    Vec3 point;
    Vec3 normal;
};

struct SweepHit {
    bool hit = false;
    EntityId entity = kInvalidEntity;
    float t = 0.0f;            // fraction in [0, 1] of the from -> to motion
    Vec3 point;
    Vec3 normal;
};

// ---------------------------------------------------------------------------
// Broadphase pair generators (free functions so they can be compared directly)
// ---------------------------------------------------------------------------

std::vector<Pair> broadphase_sweep_and_prune(const std::vector<BroadphaseBody>& bodies);
std::vector<Pair> broadphase_brute_force(const std::vector<BroadphaseBody>& bodies);

// ---------------------------------------------------------------------------
// Narrowphase primitives (exposed for tests)
// ---------------------------------------------------------------------------

bool aabb_aabb(const Aabb& a, const Aabb& b, Vec3& out_normal, float& out_penetration);
bool sphere_sphere(const Vec3& ca, float ra, const Vec3& cb, float rb, Vec3& out_normal,
                   float& out_penetration);
bool obb_obb(const Obb& a, const Obb& b, Vec3& out_normal, float& out_penetration);
bool sphere_obb(const Vec3& center, float radius, const Obb& box, Vec3& out_normal,
                float& out_penetration);

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

struct PhysicsConfig {
    Vec3 gravity = Vec3(0.0f, -9.81f, 0.0f);
    float linear_damping = 0.02f;
    float angular_damping = 0.05f;

    // Resolution.
    float restitution_slop = 1.0f;   // below this closing speed, no bounce
    float penetration_slop = 1.0e-3f;
    float baumgarte = 0.8f;
    int solver_iterations = 4;

    // Broadphase selection: sweep-and-prune (true) or brute force (false).
    bool use_sap = true;
};

// ---------------------------------------------------------------------------
// PhysicsWorld
// ---------------------------------------------------------------------------

class PhysicsWorld {
public:
    PhysicsWorld(World& world, Scene& scene);

    PhysicsConfig& config() { return config_; }
    const PhysicsConfig& config() const { return config_; }

    void set_use_sap(bool enabled) { config_.use_sap = enabled; }
    bool use_sap() const { return config_.use_sap; }

    // --- pipeline stages --------------------------------------------------
    void integrate(float dt);

    // Recomputes world transforms and cached world-space AABBs. Must run after
    // integration so contacts see current positions.
    void build_broadphase();

    std::vector<Pair> broadphase_pairs() const;

    std::vector<Contact> narrowphase(const std::vector<Pair>& pairs) const;

    void resolve_contacts(const std::vector<Contact>& contacts, float dt);

    // Convenience: the full pipeline for one fixed step.
    void step(float dt);

    // Number of contacts produced by the most recent narrowphase (for HUD/bench).
    std::size_t last_contact_count() const { return last_contact_count_; }
    std::size_t last_pair_count() const { return last_pair_count_; }
    std::size_t body_count() const { return bodies_.size(); }
    const std::vector<BroadphaseBody>& broadphase_bodies() const { return bodies_; }

    // --- queries ----------------------------------------------------------
    // `dir` is normalized internally; the returned `t` is world distance.
    RayHit raycast(const Vec3& origin, const Vec3& dir, float max_t) const;

    // Sweeps an axis-aligned box (half extents from `box`) whose center travels
    // from `from` to `to`. `t` is the fraction of that motion.
    SweepHit sweep_aabb(const Aabb& box, const Vec3& from, const Vec3& to) const;

    // Entities whose collider overlaps a sphere. Ascending id order.
    std::vector<EntityId> overlap_sphere(const Vec3& center, float radius) const;

    // --- per-entity helpers (no broadphase; used by fuzz-test references) --
    RayHit raycast_entity(EntityId id, const Vec3& origin, const Vec3& dir, float max_t) const;
    Aabb world_aabb(EntityId id) const;
    Obb world_obb(EntityId id) const;

    Vec3 position_of(EntityId id) const;
    bool is_static(EntityId id) const;
    float inverse_mass(EntityId id) const;
    float inverse_inertia(EntityId id) const;

private:
    void collect_body(EntityId id);

    World& world_;
    Scene& scene_;
    PhysicsConfig config_;

    std::vector<BroadphaseBody> bodies_;
    std::size_t last_contact_count_ = 0;
    std::size_t last_pair_count_ = 0;
};

// Sweep of an axis-aligned box (half extents `half`, center path `from`->`to`)
// against a single axis-aligned obstacle. Exposed so the fuzz suite can build a
// brute-force reference.
SweepHit sweep_aabb_vs_aabb(const Vec3& half, const Vec3& from, const Vec3& to,
                            const Aabb& obstacle, EntityId id);

}  // namespace minieng
