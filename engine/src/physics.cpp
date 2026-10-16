#include "minieng/physics.hpp"

#include <algorithm>
#include <cmath>

namespace minieng {

namespace {

constexpr float kAxisEps = 1.0e-8f;

inline Pair ordered_pair(EntityId x, EntityId y) {
    return x < y ? Pair{x, y} : Pair{y, x};
}

// --- ray / slab helpers ----------------------------------------------------

// Slab test against an axis-aligned box. `t` is a world distance in [0, max_t].
bool ray_aabb(const Vec3& o, const Vec3& d, float max_t, const Vec3& bmin, const Vec3& bmax,
              float& out_t, Vec3& out_normal) {
    float tmin = 0.0f;
    float tmax = max_t;
    Vec3 normal(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 3; ++i) {
        const float oi = o[i];
        const float di = d[i];
        if (std::fabs(di) < kAxisEps) {
            if (oi < bmin[i] || oi > bmax[i]) {
                return false;
            }
            continue;
        }
        const float inv = 1.0f / di;
        float t1 = (bmin[i] - oi) * inv;
        float t2 = (bmax[i] - oi) * inv;
        float sign = -1.0f;
        if (t1 > t2) {
            std::swap(t1, t2);
            sign = 1.0f;
        }
        if (t1 > tmin) {
            tmin = t1;
            normal = Vec3(0.0f, 0.0f, 0.0f);
            normal[i] = sign;
        }
        if (t2 < tmax) {
            tmax = t2;
        }
        if (tmin > tmax) {
            return false;
        }
    }
    out_t = tmin;
    out_normal = normal;
    return true;
}

// Analytic ray/sphere. `t` is the nearest non-negative intersection distance.
bool ray_sphere(const Vec3& o, const Vec3& d, float max_t, const Vec3& center, float radius,
                float& out_t, Vec3& out_normal) {
    const Vec3 oc = o - center;
    const float b = oc.dot(d);
    const float c = oc.length_squared() - radius * radius;
    if (c > 0.0f && b > 0.0f) {
        return false; // outside, pointing away
    }
    const float disc = b * b - c;
    if (disc < 0.0f) {
        return false;
    }
    const float sq = std::sqrt(disc);
    float t = -b - sq;
    if (t < 0.0f) {
        t = -b + sq; // origin inside
    }
    if (t < 0.0f || t > max_t) {
        return false;
    }
    out_t = t;
    out_normal = ((o + d * t) - center).normalized();
    return true;
}

// Ray vs oriented box, evaluated in the box's local frame.
bool ray_obb(const Vec3& o, const Vec3& d, float max_t, const Obb& box, float& out_t,
             Vec3& out_normal) {
    Vec3 local_o = o - box.center;
    Vec3 lo(box.axis[0].dot(local_o), box.axis[1].dot(local_o), box.axis[2].dot(local_o));
    Vec3 ld(box.axis[0].dot(d), box.axis[1].dot(d), box.axis[2].dot(d));

    float t = 0.0f;
    Vec3 local_normal(0.0f, 0.0f, 0.0f);
    float tmin = 0.0f;
    float tmax = max_t;

    for (int i = 0; i < 3; ++i) {
        const float oi = lo[i];
        const float di = ld[i];
        const float ei = box.half_extents[i];
        if (std::fabs(di) < kAxisEps) {
            if (oi < -ei || oi > ei) {
                return false;
            }
            continue;
        }
        const float inv = 1.0f / di;
        float t1 = (-ei - oi) * inv;
        float t2 = (ei - oi) * inv;
        float sign = -1.0f;
        if (t1 > t2) {
            std::swap(t1, t2);
            sign = 1.0f;
        }
        if (t1 > tmin) {
            tmin = t1;
            local_normal = Vec3(0.0f, 0.0f, 0.0f);
            local_normal[i] = sign;
        }
        if (t2 < tmax) {
            tmax = t2;
        }
        if (tmin > tmax) {
            return false;
        }
    }
    t = tmin;

    Vec3 world_normal =
        box.axis[0] * local_normal.x + box.axis[1] * local_normal.y + box.axis[2] * local_normal.z;
    out_t = t;
    out_normal = world_normal.normalized();
    return true;
}

// Extent of a collider along a world direction, used to place contact points.
float support_extent(const Collider& c, const Obb& o, const Vec3& n) {
    if (c.shape == Collider::Sphere) {
        return c.radius;
    }
    return std::fabs(n.dot(o.axis[0])) * o.half_extents.x +
           std::fabs(n.dot(o.axis[1])) * o.half_extents.y +
           std::fabs(n.dot(o.axis[2])) * o.half_extents.z;
}

// Angular contribution to the impulse denominator for scalar inertia.
float angular_term(const Vec3& r, const Vec3& n, float inv_inertia) {
    if (inv_inertia <= 0.0f) {
        return 0.0f;
    }
    const Vec3 rxn = r.cross(n);
    return n.dot((rxn * inv_inertia).cross(r));
}

} // namespace

// ---------------------------------------------------------------------------
// Aabb
// ---------------------------------------------------------------------------

void Aabb::expand(const Vec3& p) {
    min.x = std::min(min.x, p.x);
    min.y = std::min(min.y, p.y);
    min.z = std::min(min.z, p.z);
    max.x = std::max(max.x, p.x);
    max.y = std::max(max.y, p.y);
    max.z = std::max(max.z, p.z);
}

void Aabb::expand(const Aabb& other) {
    expand(other.min);
    expand(other.max);
}

Aabb Aabb::expanded(const Vec3& e) const {
    Aabb r;
    r.min = min - e;
    r.max = max + e;
    return r;
}

// ---------------------------------------------------------------------------
// Broadphase
// ---------------------------------------------------------------------------

std::vector<Pair> broadphase_sweep_and_prune(const std::vector<BroadphaseBody>& bodies) {
    struct Item {
        EntityId id;
        Aabb aabb;
    };
    std::vector<Item> items;
    items.reserve(bodies.size());
    for (const BroadphaseBody& b : bodies) {
        if (b.aabb.valid()) {
            items.push_back({b.id, b.aabb});
        }
    }
    // Sort on the X axis; ties broken by id so the order is fully deterministic.
    std::sort(items.begin(), items.end(), [](const Item& x, const Item& y) {
        if (x.aabb.min.x != y.aabb.min.x) {
            return x.aabb.min.x < y.aabb.min.x;
        }
        return x.id < y.id;
    });

    std::vector<Pair> pairs;
    for (std::size_t i = 0; i < items.size(); ++i) {
        for (std::size_t j = i + 1; j < items.size(); ++j) {
            // Sorted by min.x: once j starts past i's max, no later item can
            // overlap i either.
            if (items[j].aabb.min.x > items[i].aabb.max.x) {
                break;
            }
            if (items[i].aabb.overlaps(items[j].aabb)) {
                pairs.push_back(ordered_pair(items[i].id, items[j].id));
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& x, const Pair& y) { return x.a < y.a || (x.a == y.a && x.b < y.b); });
    return pairs;
}

std::vector<Pair> broadphase_brute_force(const std::vector<BroadphaseBody>& bodies) {
    std::vector<BroadphaseBody> live;
    live.reserve(bodies.size());
    for (const BroadphaseBody& b : bodies) {
        if (b.aabb.valid()) {
            live.push_back(b);
        }
    }
    std::sort(live.begin(), live.end(),
              [](const BroadphaseBody& x, const BroadphaseBody& y) { return x.id < y.id; });

    std::vector<Pair> pairs;
    for (std::size_t i = 0; i < live.size(); ++i) {
        for (std::size_t j = i + 1; j < live.size(); ++j) {
            if (live[i].aabb.overlaps(live[j].aabb)) {
                pairs.push_back(ordered_pair(live[i].id, live[j].id));
            }
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& x, const Pair& y) { return x.a < y.a || (x.a == y.a && x.b < y.b); });
    return pairs;
}

// ---------------------------------------------------------------------------
// Narrowphase primitives
// ---------------------------------------------------------------------------

bool aabb_aabb(const Aabb& a, const Aabb& b, Vec3& out_normal, float& out_penetration) {
    const float overlaps[3] = {
        std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x),
        std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y),
        std::min(a.max.z, b.max.z) - std::max(a.min.z, b.min.z),
    };
    if (overlaps[0] < 0.0f || overlaps[1] < 0.0f || overlaps[2] < 0.0f) {
        return false;
    }

    int axis = 0;
    if (overlaps[1] < overlaps[axis]) {
        axis = 1;
    }
    if (overlaps[2] < overlaps[axis]) {
        axis = 2;
    }

    const Vec3 ca = a.center();
    const Vec3 cb = b.center();
    Vec3 normal(0.0f, 0.0f, 0.0f);
    normal[axis] = (cb[axis] >= ca[axis]) ? 1.0f : -1.0f;

    out_normal = normal;
    out_penetration = overlaps[axis];
    return true;
}

bool sphere_sphere(const Vec3& ca, float ra, const Vec3& cb, float rb, Vec3& out_normal,
                   float& out_penetration) {
    const Vec3 delta = cb - ca;
    const float distance = delta.length();
    const float radius_sum = ra + rb;
    if (distance > radius_sum) {
        return false;
    }
    if (distance > kAxisEps) {
        out_normal = delta / distance;
    } else {
        out_normal = Vec3::unit_y();
    }
    out_penetration = radius_sum - distance;
    return true;
}

bool obb_obb(const Obb& a, const Obb& b, Vec3& out_normal, float& out_penetration) {
    Vec3 axes[15];
    for (int i = 0; i < 3; ++i) {
        axes[i] = a.axis[i];
        axes[3 + i] = b.axis[i];
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            axes[6 + i * 3 + j] = a.axis[i].cross(b.axis[j]);
        }
    }

    const Vec3 d = b.center - a.center;
    float min_penetration = 1.0e30f;
    Vec3 min_axis(0.0f, 0.0f, 0.0f);
    bool found = false;

    for (const Vec3& raw_axis : axes) {
        const float len = raw_axis.length();
        if (len < 1.0e-6f) {
            continue; // parallel edges: degenerate cross product
        }
        const Vec3 n = raw_axis / len;

        const float ra = std::fabs(n.dot(a.axis[0])) * a.half_extents.x +
                         std::fabs(n.dot(a.axis[1])) * a.half_extents.y +
                         std::fabs(n.dot(a.axis[2])) * a.half_extents.z;
        const float rb = std::fabs(n.dot(b.axis[0])) * b.half_extents.x +
                         std::fabs(n.dot(b.axis[1])) * b.half_extents.y +
                         std::fabs(n.dot(b.axis[2])) * b.half_extents.z;

        const float dist = std::fabs(d.dot(n));
        const float penetration = ra + rb - dist;
        if (penetration < 0.0f) {
            return false; // separating axis found
        }
        if (penetration < min_penetration) {
            min_penetration = penetration;
            min_axis = n;
            found = true;
        }
    }

    if (!found) {
        return false;
    }
    if (d.dot(min_axis) < 0.0f) {
        min_axis = -min_axis;
    }
    out_normal = min_axis;
    out_penetration = min_penetration;
    return true;
}

bool sphere_obb(const Vec3& center, float radius, const Obb& box, Vec3& out_normal,
                float& out_penetration) {
    const Vec3 delta = center - box.center;
    Vec3 closest = box.center;
    for (int i = 0; i < 3; ++i) {
        const float dist = delta.dot(box.axis[i]);
        const float half = box.half_extents[i];
        const float clamped = clampf(dist, -half, half);
        closest = closest + box.axis[i] * clamped;
    }

    const Vec3 diff = center - closest;
    const float dist_sq = diff.length_squared();
    if (dist_sq > radius * radius) {
        return false;
    }

    if (dist_sq > kAxisEps * kAxisEps) {
        const float dist = std::sqrt(dist_sq);
        out_normal = diff / dist; // box -> sphere
        out_penetration = radius - dist;
        return true;
    }

    // Sphere center is inside the box: push out along the nearest face.
    float best = 1.0e30f;
    Vec3 best_axis = Vec3::unit_y();
    for (int i = 0; i < 3; ++i) {
        const float dist = delta.dot(box.axis[i]);
        const float half = box.half_extents[i];
        const float penetration = half - std::fabs(dist);
        if (penetration < best) {
            best = penetration;
            best_axis = box.axis[i] * (dist >= 0.0f ? 1.0f : -1.0f);
        }
    }
    out_normal = best_axis;
    out_penetration = best + radius;
    return true;
}

// ---------------------------------------------------------------------------
// PhysicsWorld
// ---------------------------------------------------------------------------

PhysicsWorld::PhysicsWorld(World& world, Scene& scene) : world_(world), scene_(scene) {}

void PhysicsWorld::collect_body(EntityId id) {
    if (world_.get_collider(id) == nullptr) {
        return;
    }
    BroadphaseBody body;
    body.id = id;
    body.aabb = world_aabb(id);
    if (body.aabb.valid()) {
        bodies_.push_back(body);
    }
}

void PhysicsWorld::integrate(float dt) {
    if (!(dt > 0.0f)) {
        return;
    }
    const Vec3 gravity = config_.gravity;
    const float linear_damp = clampf(1.0f - config_.linear_damping * dt, 0.0f, 1.0f);
    const float angular_damp = clampf(1.0f - config_.angular_damping * dt, 0.0f, 1.0f);

    for (const EntityId id : world_.entities()) {
        RigidBody* body = world_.get_rigid_body(id);
        if (body == nullptr || body->is_static) {
            continue;
        }
        TransformRef* ref = world_.get_transform_ref(id);
        if (ref == nullptr || ref->scene_node < 0) {
            continue;
        }

        // Semi-implicit Euler: velocity first, then position.
        body->velocity = (body->velocity + gravity * dt) * linear_damp;
        body->angular_velocity = body->angular_velocity * angular_damp;

        Transform& t = scene_.local_of(ref->scene_node);
        t.position = t.position + body->velocity * dt;

        // Angular integration via the quaternion derivative:
        //   dq/dt = 0.5 * (0, omega) * q
        const Vec3& w = body->angular_velocity;
        if (w.length_squared() > kEpsilonSq) {
            const Quat omega(0.0f, w.x, w.y, w.z);
            const Quat dq = (omega * t.rotation) * 0.5f;
            t.rotation = (t.rotation + dq * dt).normalize();
        }
    }
}

void PhysicsWorld::build_broadphase() {
    scene_.update_world_transforms();
    bodies_.clear();
    bodies_.reserve(world_.entity_count());
    for (const EntityId id : world_.entities()) {
        collect_body(id);
    }
}

std::vector<Pair> PhysicsWorld::broadphase_pairs() const {
    return config_.use_sap ? broadphase_sweep_and_prune(bodies_) : broadphase_brute_force(bodies_);
}

std::vector<Contact> PhysicsWorld::narrowphase(const std::vector<Pair>& pairs) const {
    std::vector<Contact> contacts;
    contacts.reserve(pairs.size());

    for (const Pair& pair : pairs) {
        const Collider* ca = world_.get_collider(pair.a);
        const Collider* cb = world_.get_collider(pair.b);
        if (ca == nullptr || cb == nullptr) {
            continue;
        }

        const Vec3 center_a = position_of(pair.a);
        const Vec3 center_b = position_of(pair.b);

        Vec3 normal(0.0f, 0.0f, 0.0f);
        float penetration = 0.0f;
        bool hit = false;

        const bool a_sphere = ca->shape == Collider::Sphere;
        const bool b_sphere = cb->shape == Collider::Sphere;

        if (a_sphere && b_sphere) {
            hit = sphere_sphere(center_a, ca->radius, center_b, cb->radius, normal, penetration);
        } else if (!a_sphere && !b_sphere) {
            const Obb oa = world_obb(pair.a);
            const Obb ob = world_obb(pair.b);
            if (ca->shape == Collider::AABB && cb->shape == Collider::AABB) {
                hit = aabb_aabb(world_aabb(pair.a), world_aabb(pair.b), normal, penetration);
            } else {
                hit = obb_obb(oa, ob, normal, penetration);
            }
        } else if (a_sphere) {
            // sphere_obb reports box -> sphere; we want a -> b.
            hit = sphere_obb(center_a, ca->radius, world_obb(pair.b), normal, penetration);
            normal = -normal;
        } else {
            hit = sphere_obb(center_b, cb->radius, world_obb(pair.a), normal, penetration);
        }

        if (!hit) {
            continue;
        }

        Contact contact;
        contact.a = pair.a;
        contact.b = pair.b;
        contact.normal = normal;
        contact.penetration = penetration;

        const Obb oa = world_obb(pair.a);
        const Obb ob = world_obb(pair.b);
        const float proj_a = support_extent(*ca, oa, normal);
        const float proj_b = support_extent(*cb, ob, normal);
        contact.point = ((center_a + normal * proj_a) + (center_b - normal * proj_b)) * 0.5f;

        contacts.push_back(contact);
    }
    return contacts;
}

void PhysicsWorld::resolve_contacts(const std::vector<Contact>& contacts, float dt) {
    (void)dt;

    for (const Contact& contact : contacts) {
        RigidBody* body_a = world_.get_rigid_body(contact.a);
        RigidBody* body_b = world_.get_rigid_body(contact.b);
        if (body_a == nullptr || body_b == nullptr) {
            continue;
        }

        const float inv_ma = inverse_mass(contact.a);
        const float inv_mb = inverse_mass(contact.b);
        const float inv_sum = inv_ma + inv_mb;
        if (inv_sum <= 0.0f) {
            continue; // static vs static
        }

        const float inv_ia = inverse_inertia(contact.a);
        const float inv_ib = inverse_inertia(contact.b);

        TransformRef* ref_a = world_.get_transform_ref(contact.a);
        TransformRef* ref_b = world_.get_transform_ref(contact.b);
        Transform* ta =
            (ref_a && ref_a->scene_node >= 0) ? &scene_.local_of(ref_a->scene_node) : nullptr;
        Transform* tb =
            (ref_b && ref_b->scene_node >= 0) ? &scene_.local_of(ref_b->scene_node) : nullptr;

        const Vec3& n = contact.normal;
        const Vec3 point = contact.point;

        // --- positional correction (Baumgarte with slop) ------------------
        const float correction = contact.penetration - config_.penetration_slop;
        if (correction > 0.0f) {
            const Vec3 push = n * (correction * config_.baumgarte / inv_sum);
            if (ta != nullptr && inv_ma > 0.0f) {
                ta->position = ta->position - push * inv_ma;
            }
            if (tb != nullptr && inv_mb > 0.0f) {
                tb->position = tb->position + push * inv_mb;
            }
        }

        const Vec3 pa = ta ? ta->position : position_of(contact.a);
        const Vec3 pb = tb ? tb->position : position_of(contact.b);

        const float restitution = std::max(body_a->restitution, body_b->restitution);

        const int iterations = std::max(1, config_.solver_iterations);
        for (int iter = 0; iter < iterations; ++iter) {
            const Vec3 ra = point - (ta ? ta->position : pa);
            const Vec3 rb = point - (tb ? tb->position : pb);

            const Vec3 va = body_a->velocity + body_a->angular_velocity.cross(ra);
            const Vec3 vb = body_b->velocity + body_b->angular_velocity.cross(rb);
            const Vec3 vrel = vb - va;

            const float vn = vrel.dot(n);
            if (vn >= 0.0f) {
                break; // separating
            }

            // No restitution for slow closing speeds, to avoid resting jitter.
            const float e = (iter == 0 && -vn > config_.restitution_slop) ? restitution : 0.0f;

            const float denom = inv_sum + angular_term(ra, n, inv_ia) + angular_term(rb, n, inv_ib);
            if (denom <= 0.0f) {
                break;
            }
            const float j = -(1.0f + e) * vn / denom;
            if (j <= 0.0f) {
                break;
            }
            const Vec3 impulse = n * j;

            body_a->velocity = body_a->velocity - impulse * inv_ma;
            body_b->velocity = body_b->velocity + impulse * inv_mb;
            body_a->angular_velocity = body_a->angular_velocity - (ra.cross(impulse)) * inv_ia;
            body_b->angular_velocity = body_b->angular_velocity + (rb.cross(impulse)) * inv_ib;

            // --- Coulomb friction on the tangent --------------------------
            const Vec3 va2 = body_a->velocity + body_a->angular_velocity.cross(ra);
            const Vec3 vb2 = body_b->velocity + body_b->angular_velocity.cross(rb);
            const Vec3 vrel2 = vb2 - va2;
            const Vec3 tangential = vrel2 - n * vrel2.dot(n);
            const float tangent_len = tangential.length();
            if (tangent_len > 1.0e-5f) {
                const Vec3 t = tangential / tangent_len;
                const float denom_t =
                    inv_sum + angular_term(ra, t, inv_ia) + angular_term(rb, t, inv_ib);
                if (denom_t > 0.0f) {
                    float jt = -vrel2.dot(t) / denom_t;
                    const float mu = std::sqrt(std::max(0.0f, body_a->friction) *
                                               std::max(0.0f, body_b->friction));
                    const float max_jt = mu * j;
                    jt = clampf(jt, -max_jt, max_jt);
                    const Vec3 fimpulse = t * jt;
                    body_a->velocity = body_a->velocity - fimpulse * inv_ma;
                    body_b->velocity = body_b->velocity + fimpulse * inv_mb;
                    body_a->angular_velocity =
                        body_a->angular_velocity - (ra.cross(fimpulse)) * inv_ia;
                    body_b->angular_velocity =
                        body_b->angular_velocity + (rb.cross(fimpulse)) * inv_ib;
                }
            }
        }
    }
}

void PhysicsWorld::step(float dt) {
    integrate(dt);
    build_broadphase();
    const std::vector<Pair> pairs = broadphase_pairs();
    last_pair_count_ = pairs.size();
    const std::vector<Contact> contacts = narrowphase(pairs);
    last_contact_count_ = contacts.size();
    resolve_contacts(contacts, dt);
    scene_.update_world_transforms();
}

// ---------------------------------------------------------------------------
// Per-entity helpers
// ---------------------------------------------------------------------------

Vec3 PhysicsWorld::position_of(EntityId id) const {
    const TransformRef* ref = world_.get_transform_ref(id);
    if (ref == nullptr || ref->scene_node < 0 || !scene_.valid(ref->scene_node)) {
        return Vec3::zero();
    }
    return scene_.world_position_of(ref->scene_node);
}

Aabb PhysicsWorld::world_aabb(EntityId id) const {
    Aabb box;
    const Collider* collider = world_.get_collider(id);
    if (collider == nullptr) {
        return box;
    }
    const Vec3 center = position_of(id);

    switch (collider->shape) {
        case Collider::Sphere: {
            const Vec3 r(collider->radius, collider->radius, collider->radius);
            box.min = center - r;
            box.max = center + r;
            break;
        }
        case Collider::AABB: {
            // Axis-aligned by definition: rotation is intentionally ignored.
            box.min = center - collider->half_extents;
            box.max = center + collider->half_extents;
            break;
        }
        case Collider::OBB: {
            // Bounding box of a rotated box: centre +/- sum(|axis_i| * h_i).
            // (Adding only the six face points would under-estimate the extent.)
            const Obb obb = world_obb(id);
            Vec3 r(0.0f, 0.0f, 0.0f);
            for (int i = 0; i < 3; ++i) {
                const Vec3 a = obb.axis[i];
                const float h = obb.half_extents[i];
                r = r + Vec3(std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)) * h;
            }
            box.min = center - r;
            box.max = center + r;
            break;
        }
    }
    return box;
}

Obb PhysicsWorld::world_obb(EntityId id) const {
    Obb obb;
    const Collider* collider = world_.get_collider(id);
    if (collider == nullptr) {
        return obb;
    }
    obb.center = position_of(id);
    obb.half_extents = collider->half_extents;

    const TransformRef* ref = world_.get_transform_ref(id);
    if (collider->shape == Collider::AABB || ref == nullptr || ref->scene_node < 0) {
        return obb; // identity axes
    }

    const Mat4& world = scene_.world_of(ref->scene_node);
    obb.axis[0] = Vec3(world.at(0, 0), world.at(1, 0), world.at(2, 0)).normalized();
    obb.axis[1] = Vec3(world.at(0, 1), world.at(1, 1), world.at(2, 1)).normalized();
    obb.axis[2] = Vec3(world.at(0, 2), world.at(1, 2), world.at(2, 2)).normalized();
    if (obb.axis[0].length_squared() < kEpsilonSq) {
        obb.axis[0] = Vec3::unit_x();
    }
    if (obb.axis[1].length_squared() < kEpsilonSq) {
        obb.axis[1] = Vec3::unit_y();
    }
    if (obb.axis[2].length_squared() < kEpsilonSq) {
        obb.axis[2] = Vec3::unit_z();
    }
    return obb;
}

bool PhysicsWorld::is_static(EntityId id) const {
    const RigidBody* body = world_.get_rigid_body(id);
    return body == nullptr || body->is_static;
}

float PhysicsWorld::inverse_mass(EntityId id) const {
    const RigidBody* body = world_.get_rigid_body(id);
    return body == nullptr ? 0.0f : body->inverse_mass();
}

float PhysicsWorld::inverse_inertia(EntityId id) const {
    const RigidBody* body = world_.get_rigid_body(id);
    if (body == nullptr || body->is_static || body->mass <= 0.0f) {
        return 0.0f;
    }
    const Collider* collider = world_.get_collider(id);

    float inertia = 0.1f * body->mass;
    if (collider != nullptr) {
        if (collider->shape == Collider::Sphere) {
            inertia = 0.4f * body->mass * collider->radius * collider->radius;
        } else {
            // Scalar approximation of the box inertia diagonal.
            inertia = body->mass * collider->half_extents.length_squared() / 3.0f;
        }
    }
    return inertia > kEpsilon ? 1.0f / inertia : 0.0f;
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

RayHit PhysicsWorld::raycast_entity(EntityId id, const Vec3& origin, const Vec3& dir,
                                    float max_t) const {
    RayHit result;
    const Collider* collider = world_.get_collider(id);
    if (collider == nullptr) {
        return result;
    }

    float t = 0.0f;
    Vec3 normal(0.0f, 0.0f, 0.0f);
    bool hit = false;

    switch (collider->shape) {
        case Collider::Sphere:
            hit = ray_sphere(origin, dir, max_t, position_of(id), collider->radius, t, normal);
            break;
        case Collider::AABB: {
            const Aabb box = world_aabb(id);
            hit = ray_aabb(origin, dir, max_t, box.min, box.max, t, normal);
            break;
        }
        case Collider::OBB: hit = ray_obb(origin, dir, max_t, world_obb(id), t, normal); break;
    }

    if (hit) {
        result.hit = true;
        result.entity = id;
        result.t = t;
        result.point = origin + dir * t;
        result.normal = normal;
    }
    return result;
}

RayHit PhysicsWorld::raycast(const Vec3& origin, const Vec3& dir, float max_t) const {
    RayHit best;
    best.t = max_t;
    if (max_t <= 0.0f) {
        return best;
    }
    const Vec3 d = dir.normalized();
    if (d.length_squared() < kEpsilonSq) {
        return best;
    }

    for (const BroadphaseBody& body : bodies_) {
        // Cheap AABB reject before the exact per-shape test.
        float t = 0.0f;
        Vec3 n;
        if (!ray_aabb(origin, d, best.t, body.aabb.min, body.aabb.max, t, n)) {
            continue;
        }
        const RayHit hit = raycast_entity(body.id, origin, d, best.t);
        if (hit.hit && hit.t < best.t) {
            best = hit;
        }
    }
    return best;
}

SweepHit sweep_aabb_vs_aabb(const Vec3& half, const Vec3& from, const Vec3& to,
                            const Aabb& obstacle, EntityId id) {
    SweepHit result;
    const Vec3 delta = to - from;
    const float length = delta.length();
    if (length < 1.0e-9f) {
        return result;
    }
    const Vec3 dir = delta / length;

    // Minkowski expansion: inflating the obstacle by the moving box's half
    // extents turns the swept-box test into a single ray/slab test, which is
    // exact for axis-aligned geometry and cannot tunnel.
    const Aabb expanded = obstacle.expanded(half);

    float t = 0.0f;
    Vec3 normal(0.0f, 0.0f, 0.0f);
    if (!ray_aabb(from, dir, length, expanded.min, expanded.max, t, normal)) {
        return result;
    }

    result.hit = true;
    result.entity = id;
    result.t = t / length;
    result.point = from + dir * t;
    result.normal = normal;
    return result;
}

SweepHit PhysicsWorld::sweep_aabb(const Aabb& box, const Vec3& from, const Vec3& to) const {
    SweepHit best;
    best.t = 1.0f;

    const Vec3 half = box.extent();
    const Vec3 delta = to - from;
    if (delta.length() < 1.0e-9f) {
        return best;
    }

    for (const BroadphaseBody& body : bodies_) {
        // Broadphase reject: the swept AABB of the moving box.
        Aabb swept;
        swept.min =
            Vec3(std::min(from.x, to.x), std::min(from.y, to.y), std::min(from.z, to.z)) - half;
        swept.max =
            Vec3(std::max(from.x, to.x), std::max(from.y, to.y), std::max(from.z, to.z)) + half;
        if (!swept.overlaps(body.aabb)) {
            continue;
        }
        const SweepHit hit = sweep_aabb_vs_aabb(half, from, to, body.aabb, body.id);
        if (hit.hit && hit.t < best.t) {
            best = hit;
        }
    }
    return best;
}

std::vector<EntityId> PhysicsWorld::overlap_sphere(const Vec3& center, float radius) const {
    std::vector<EntityId> result;
    const Vec3 r(radius, radius, radius);
    Aabb sphere_box;
    sphere_box.min = center - r;
    sphere_box.max = center + r;

    for (const BroadphaseBody& body : bodies_) {
        if (!sphere_box.overlaps(body.aabb)) {
            continue;
        }
        const Collider* collider = world_.get_collider(body.id);
        if (collider == nullptr) {
            continue;
        }
        Vec3 n;
        float pen = 0.0f;
        bool hit = false;
        if (collider->shape == Collider::Sphere) {
            hit = sphere_sphere(center, radius, position_of(body.id), collider->radius, n, pen);
        } else {
            hit = sphere_obb(center, radius, world_obb(body.id), n, pen);
        }
        if (hit) {
            result.push_back(body.id);
        }
    }
    return result;
}

} // namespace minieng
