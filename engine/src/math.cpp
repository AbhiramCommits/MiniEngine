#include "minieng/math.hpp"

namespace minieng {

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------

Vec3 Vec3::normalized(float eps) const {
    const float len = length();
    if (len <= eps) {
        return Vec3::zero();
    }
    const float inv = 1.0f / len;
    return Vec3(x * inv, y * inv, z * inv);
}

// ---------------------------------------------------------------------------
// Quat
// ---------------------------------------------------------------------------

Quat Quat::from_axis_angle(const Vec3& axis, float angle) {
    const Vec3 a = axis.normalized();
    // Degenerate axis: no rotation is representable, return identity.
    if (a.length_squared() <= kEpsilonSq) {
        return Quat::identity();
    }
    const float half = 0.5f * angle;
    const float s = std::sin(half);
    return Quat(std::cos(half), a.x * s, a.y * s, a.z * s);
}

Quat Quat::from_euler(float rx, float ry, float rz) {
    const Quat qx = from_axis_angle(Vec3::unit_x(), rx);
    const Quat qy = from_axis_angle(Vec3::unit_y(), ry);
    const Quat qz = from_axis_angle(Vec3::unit_z(), rz);
    return qz.multiply(qy).multiply(qx);
}

Quat Quat::multiply(const Quat& o) const {
    return Quat(w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w);
}

Quat Quat::normalize(float eps) const {
    const float len = length();
    if (len <= eps) {
        return Quat::identity();
    }
    const float inv = 1.0f / len;
    return Quat(w * inv, x * inv, y * inv, z * inv);
}

Vec3 Quat::rotate_vector(const Vec3& v) const {
    const Vec3 u(x, y, z);
    const Vec3 t = u.cross(v) * 2.0f;
    return v + t * w + u.cross(t);
}

Quat slerp(const Quat& a, const Quat& b, float t) {
    Quat bb = b;
    float d = a.dot(b);

    // Shortest arc: flip b when the quaternions are in opposite hemispheres so
    // the interpolation takes the short way around the sphere.
    if (d < 0.0f) {
        bb = Quat(-b.w, -b.x, -b.y, -b.z);
        d = -d;
    }

    // Near-parallel: sin(theta) would be ~0, so use a linear fallback.
    if (d > 1.0f - 1.0e-5f) {
        const Quat r = a * (1.0f - t) + bb * t;
        return r.normalize();
    }

    d = clampf(d, -1.0f, 1.0f);
    const float theta = std::acos(d);
    const float sin_theta = std::sin(theta);
    const float wa = std::sin((1.0f - t) * theta) / sin_theta;
    const float wb = std::sin(t * theta) / sin_theta;
    return (a * wa + bb * wb).normalize();
}

// ---------------------------------------------------------------------------
// Mat4
// ---------------------------------------------------------------------------

Mat4 Mat4::identity() {
    return Mat4();
}

Mat4 Mat4::from_translation(const Vec3& t) {
    Mat4 r;
    r.at(0, 3) = t.x;
    r.at(1, 3) = t.y;
    r.at(2, 3) = t.z;
    return r;
}

Mat4 Mat4::from_quat(const Quat& qq) {
    const Quat q = qq.normalize();
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;

    Mat4 r;
    r.at(0, 0) = 1.0f - 2.0f * (yy + zz);
    r.at(0, 1) = 2.0f * (xy - wz);
    r.at(0, 2) = 2.0f * (xz + wy);
    r.at(1, 0) = 2.0f * (xy + wz);
    r.at(1, 1) = 1.0f - 2.0f * (xx + zz);
    r.at(1, 2) = 2.0f * (yz - wx);
    r.at(2, 0) = 2.0f * (xz - wy);
    r.at(2, 1) = 2.0f * (yz + wx);
    r.at(2, 2) = 1.0f - 2.0f * (xx + yy);
    return r;
}

Mat4 Mat4::from_scale(const Vec3& s) {
    Mat4 r;
    r.at(0, 0) = s.x;
    r.at(1, 1) = s.y;
    r.at(2, 2) = s.z;
    return r;
}

Mat4 Mat4::from_trs(const Vec3& t, const Quat& r, const Vec3& s) {
    return from_translation(t) * from_quat(r) * from_scale(s);
}

void Mat4::decompose(const Mat4& mat, Vec3& out_t, Quat& out_r, Vec3& out_s) {
    out_t = Vec3(mat.at(0, 3), mat.at(1, 3), mat.at(2, 3));

    Vec3 c0(mat.at(0, 0), mat.at(1, 0), mat.at(2, 0));
    Vec3 c1(mat.at(0, 1), mat.at(1, 1), mat.at(2, 1));
    Vec3 c2(mat.at(0, 2), mat.at(1, 2), mat.at(2, 2));

    float sx = c0.length();
    float sy = c1.length();
    float sz = c2.length();

    // Determinant sign tells us whether the basis is mirrored; fold the sign
    // into the first axis so the extracted rotation stays a proper rotation.
    const float det = c0.dot(c1.cross(c2));
    if (det < 0.0f) {
        sx = -sx;
    }

    out_s = Vec3(sx, sy, sz);

    if (std::fabs(sx) > kEpsilon)
        c0 = c0 / sx;
    if (std::fabs(sy) > kEpsilon)
        c1 = c1 / sy;
    if (std::fabs(sz) > kEpsilon)
        c2 = c2 / sz;

    Mat4 rot;
    rot.at(0, 0) = c0.x;
    rot.at(1, 0) = c0.y;
    rot.at(2, 0) = c0.z;
    rot.at(0, 1) = c1.x;
    rot.at(1, 1) = c1.y;
    rot.at(2, 1) = c1.z;
    rot.at(0, 2) = c2.x;
    rot.at(1, 2) = c2.y;
    rot.at(2, 2) = c2.z;

    const float trace = rot.at(0, 0) + rot.at(1, 1) + rot.at(2, 2);
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f; // s = 4w
        out_r.w = 0.25f * s;
        out_r.x = (rot.at(2, 1) - rot.at(1, 2)) / s;
        out_r.y = (rot.at(0, 2) - rot.at(2, 0)) / s;
        out_r.z = (rot.at(1, 0) - rot.at(0, 1)) / s;
    } else if (rot.at(0, 0) > rot.at(1, 1) && rot.at(0, 0) > rot.at(2, 2)) {
        const float s = std::sqrt(1.0f + rot.at(0, 0) - rot.at(1, 1) - rot.at(2, 2)) * 2.0f;
        out_r.w = (rot.at(2, 1) - rot.at(1, 2)) / s;
        out_r.x = 0.25f * s;
        out_r.y = (rot.at(0, 1) + rot.at(1, 0)) / s;
        out_r.z = (rot.at(0, 2) + rot.at(2, 0)) / s;
    } else if (rot.at(1, 1) > rot.at(2, 2)) {
        const float s = std::sqrt(1.0f + rot.at(1, 1) - rot.at(0, 0) - rot.at(2, 2)) * 2.0f;
        out_r.w = (rot.at(0, 2) - rot.at(2, 0)) / s;
        out_r.x = (rot.at(0, 1) + rot.at(1, 0)) / s;
        out_r.y = 0.25f * s;
        out_r.z = (rot.at(1, 2) + rot.at(2, 1)) / s;
    } else {
        const float s = std::sqrt(1.0f + rot.at(2, 2) - rot.at(0, 0) - rot.at(1, 1)) * 2.0f;
        out_r.w = (rot.at(1, 0) - rot.at(0, 1)) / s;
        out_r.x = (rot.at(0, 2) + rot.at(2, 0)) / s;
        out_r.y = (rot.at(1, 2) + rot.at(2, 1)) / s;
        out_r.z = 0.25f * s;
    }
    out_r = out_r.normalize();
}

Mat4 Mat4::multiply(const Mat4& o) const {
    Mat4 r;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += at(row, k) * o.at(k, col);
            }
            r.at(row, col) = sum;
        }
    }
    return r;
}

Vec3 Mat4::transform_point(const Vec3& p) const {
    return Vec3(at(0, 0) * p.x + at(0, 1) * p.y + at(0, 2) * p.z + at(0, 3),
                at(1, 0) * p.x + at(1, 1) * p.y + at(1, 2) * p.z + at(1, 3),
                at(2, 0) * p.x + at(2, 1) * p.y + at(2, 2) * p.z + at(2, 3));
}

Vec3 Mat4::transform_direction(const Vec3& d) const {
    return Vec3(at(0, 0) * d.x + at(0, 1) * d.y + at(0, 2) * d.z,
                at(1, 0) * d.x + at(1, 1) * d.y + at(1, 2) * d.z,
                at(2, 0) * d.x + at(2, 1) * d.y + at(2, 2) * d.z);
}

Mat4 Mat4::perspective(float fov_y_radians, float aspect, float near_z, float far_z) {
    Mat4 r;
    for (float& value : r.m) {
        value = 0.0f;
    }
    const float f = 1.0f / std::tan(fov_y_radians * 0.5f);
    r.at(0, 0) = f / (aspect > kEpsilon ? aspect : 1.0f);
    r.at(1, 1) = f;
    r.at(2, 2) = (far_z + near_z) / (near_z - far_z);
    r.at(2, 3) = (2.0f * far_z * near_z) / (near_z - far_z);
    r.at(3, 2) = -1.0f;
    return r;
}

Mat4 Mat4::look_at(const Vec3& eye, const Vec3& target, const Vec3& up) {
    const Vec3 forward = (target - eye).normalized();
    Vec3 right = forward.cross(up).normalized();
    if (right.length_squared() < kEpsilonSq) {
        right = Vec3::unit_x();
    }
    const Vec3 true_up = right.cross(forward);

    Mat4 r;
    r.at(0, 0) = right.x;
    r.at(0, 1) = right.y;
    r.at(0, 2) = right.z;
    r.at(0, 3) = -right.dot(eye);
    r.at(1, 0) = true_up.x;
    r.at(1, 1) = true_up.y;
    r.at(1, 2) = true_up.z;
    r.at(1, 3) = -true_up.dot(eye);
    r.at(2, 0) = -forward.x;
    r.at(2, 1) = -forward.y;
    r.at(2, 2) = -forward.z;
    r.at(2, 3) = forward.dot(eye);
    return r;
}

Mat4 Mat4::transposed() const {
    Mat4 r;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            r.at(row, col) = at(col, row);
        }
    }
    return r;
}

Mat4 Mat4::inverse_affine() const {
    const float a00 = at(0, 0), a01 = at(0, 1), a02 = at(0, 2);
    const float a10 = at(1, 0), a11 = at(1, 1), a12 = at(1, 2);
    const float a20 = at(2, 0), a21 = at(2, 1), a22 = at(2, 2);

    const float det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
                      a02 * (a10 * a21 - a11 * a20);

    Mat4 r;
    if (std::fabs(det) <= kEpsilon) {
        // Singular: no meaningful inverse, hand back identity.
        return r;
    }
    const float inv = 1.0f / det;

    r.at(0, 0) = (a11 * a22 - a12 * a21) * inv;
    r.at(0, 1) = (a02 * a21 - a01 * a22) * inv;
    r.at(0, 2) = (a01 * a12 - a02 * a11) * inv;
    r.at(1, 0) = (a12 * a20 - a10 * a22) * inv;
    r.at(1, 1) = (a00 * a22 - a02 * a20) * inv;
    r.at(1, 2) = (a02 * a10 - a00 * a12) * inv;
    r.at(2, 0) = (a10 * a21 - a11 * a20) * inv;
    r.at(2, 1) = (a01 * a20 - a00 * a21) * inv;
    r.at(2, 2) = (a00 * a11 - a01 * a10) * inv;

    const Vec3 t(at(0, 3), at(1, 3), at(2, 3));
    r.at(0, 3) = -(r.at(0, 0) * t.x + r.at(0, 1) * t.y + r.at(0, 2) * t.z);
    r.at(1, 3) = -(r.at(1, 0) * t.x + r.at(1, 1) * t.y + r.at(1, 2) * t.z);
    r.at(2, 3) = -(r.at(2, 0) * t.x + r.at(2, 1) * t.y + r.at(2, 2) * t.z);
    return r;
}

} // namespace minieng
