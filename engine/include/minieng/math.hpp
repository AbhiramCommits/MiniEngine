#pragma once

// Hand-rolled linear algebra for MiniEngine.
//
// Conventions (kept consistent everywhere in the engine):
//   * Vec3 is a column vector.
//   * Quat is (w, x, y, z) and rotates column vectors: v' = q * v * q^-1.
//   * Mat4 stores 16 floats row-major: m[row * 4 + col].
//   * Transforms compound left-to-right: world = parent * local, p' = M * p.
//   * A quaternion composed as a * b means "apply b, then a".
//
// Nothing here depends on a windowing or rendering library.

#include <cmath>

namespace minieng {

// ---------------------------------------------------------------------------
// Constants and scalar helpers
// ---------------------------------------------------------------------------

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kHalfPi = 1.57079632679489661923f;

constexpr float kEpsilon = 1.0e-6f;
constexpr float kEpsilonSq = kEpsilon * kEpsilon;

constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

constexpr inline float deg_to_rad(float d) {
    return d * kDegToRad;
}
constexpr inline float rad_to_deg(float r) {
    return r * kRadToDeg;
}

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline float lerpf(float a, float b, float t) {
    return a + (b - a) * t;
}

inline bool approx_equal(float a, float b, float eps = kEpsilon) {
    return std::fabs(a - b) <= eps;
}

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    static Vec3 zero() { return Vec3(0.0f, 0.0f, 0.0f); }
    static Vec3 one() { return Vec3(1.0f, 1.0f, 1.0f); }
    static Vec3 unit_x() { return Vec3(1.0f, 0.0f, 0.0f); }
    static Vec3 unit_y() { return Vec3(0.0f, 1.0f, 0.0f); }
    static Vec3 unit_z() { return Vec3(0.0f, 0.0f, 1.0f); }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }

    Vec3& operator+=(const Vec3& o) {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    Vec3& operator-=(const Vec3& o) {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    Vec3& operator*=(float s) {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }

    float length_squared() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(length_squared()); }

    // Returns the zero vector for inputs shorter than `eps` instead of NaN.
    Vec3 normalized(float eps = kEpsilon) const;

    Vec3 scale(float s) const { return *this * s; }

    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }

    bool approx_equal_to(const Vec3& o, float eps = kEpsilon) const {
        return std::fabs(x - o.x) <= eps && std::fabs(y - o.y) <= eps && std::fabs(z - o.z) <= eps;
    }
};

inline Vec3 operator*(float s, const Vec3& v) {
    return v * s;
}

// ---------------------------------------------------------------------------
// Quat
// ---------------------------------------------------------------------------

struct Quat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat identity() { return Quat(1.0f, 0.0f, 0.0f, 0.0f); }

    // axis is normalized internally; angle is in radians.
    static Quat from_axis_angle(const Vec3& axis, float angle);

    // Intrinsic composition applying X first, then Y, then Z (q = qz * qy * qx).
    static Quat from_euler(float rx, float ry, float rz);

    // Hamilton product: (this * o) applies `o` first, then `this`.
    Quat multiply(const Quat& o) const;

    Quat operator*(const Quat& o) const { return multiply(o); }

    Quat operator+(const Quat& o) const { return Quat(w + o.w, x + o.x, y + o.y, z + o.z); }
    Quat operator-(const Quat& o) const { return Quat(w - o.w, x - o.x, y - o.y, z - o.z); }
    Quat operator*(float s) const { return Quat(w * s, x * s, y * s, z * s); }

    Vec3 vector_part() const { return Vec3(x, y, z); }

    float dot(const Quat& o) const { return w * o.w + x * o.x + y * o.y + z * o.z; }

    float length() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    // Falls back to identity for degenerate (near-zero) quaternions.
    Quat normalize(float eps = kEpsilon) const;

    Quat conjugate() const { return Quat(w, -x, -y, -z); }

    // Assumes a unit quaternion. v' = v + 2w(u x v) + 2(u x (u x v)).
    Vec3 rotate_vector(const Vec3& v) const;

    bool approx_equal_to(const Quat& o, float eps = kEpsilon) const {
        // q and -q represent the same rotation.
        if (std::fabs(w - o.w) <= eps && std::fabs(x - o.x) <= eps && std::fabs(y - o.y) <= eps &&
            std::fabs(z - o.z) <= eps) {
            return true;
        }
        return std::fabs(w + o.w) <= eps && std::fabs(x + o.x) <= eps &&
               std::fabs(y + o.y) <= eps && std::fabs(z + o.z) <= eps;
    }
};

// Shortest-arc spherical interpolation. Near-parallel inputs (|dot| > 1 - 1e-5)
// fall back to a normalized component-wise lerp to avoid a divide-by-zero.
Quat slerp(const Quat& a, const Quat& b, float t);

inline Quat operator*(float s, const Quat& q) {
    return q * s;
}

// ---------------------------------------------------------------------------
// Mat4
// ---------------------------------------------------------------------------

struct Mat4 {
    // Row-major: m[row * 4 + col].
    float m[16] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

    Mat4() = default;

    static Mat4 identity();
    static Mat4 from_translation(const Vec3& t);
    static Mat4 from_quat(const Quat& q);
    static Mat4 from_scale(const Vec3& s);

    // Compose translate * rotate * scale.
    static Mat4 from_trs(const Vec3& t, const Quat& r, const Vec3& s);

    // Splits an affine TRS matrix back into components.
    static void decompose(const Mat4& mat, Vec3& out_t, Quat& out_r, Vec3& out_s);

    // Right-handed camera helpers (looking down -Z in view space).
    static Mat4 perspective(float fov_y_radians, float aspect, float near_z, float far_z);
    static Mat4 look_at(const Vec3& eye, const Vec3& target, const Vec3& up);

    float at(int row, int col) const { return m[row * 4 + col]; }
    float& at(int row, int col) { return m[row * 4 + col]; }

    // Standard matrix product: (this * o) applies `o` first, then `this`.
    Mat4 multiply(const Mat4& o) const;
    Mat4 operator*(const Mat4& o) const { return multiply(o); }

    Vec3 transform_point(const Vec3& p) const;
    Vec3 transform_direction(const Vec3& d) const;

    // Inverse of an affine transform (bottom row assumed [0 0 0 1]).
    Mat4 inverse_affine() const;

    Mat4 transposed() const;
};

} // namespace minieng
