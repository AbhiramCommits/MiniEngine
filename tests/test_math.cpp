// Math library tests: quaternion/matrix equivalence, slerp behaviour,
// affine inverse round trips, composition and rotation orthonormality.

#include <gtest/gtest.h>

#include <vector>

#include "minieng/math.hpp"

using namespace minieng;

namespace {

// A handful of fixed, non-trivial rotations used across the equivalence tests.
struct RotationCase {
    const char* name;
    Quat q;
};

std::vector<RotationCase> rotation_cases() {
    return {
        {"identity", Quat::identity()},
        {"yaw90", Quat::from_axis_angle(Vec3::unit_y(), kHalfPi)},
        {"axis_xyz_30", Quat::from_axis_angle(Vec3(1.0f, 2.0f, -0.5f), deg_to_rad(30.0f))},
        {"euler", Quat::from_euler(deg_to_rad(20.0f), deg_to_rad(-45.0f), deg_to_rad(70.0f))},
        {"axis_neg_z_135", Quat::from_axis_angle(Vec3(0.0f, 0.0f, -1.0f), deg_to_rad(135.0f))},
    };
}

}  // namespace

TEST(Math, Vec3Basics) {
    const Vec3 a(1.0f, 2.0f, 3.0f);
    const Vec3 b(-4.0f, 5.0f, 0.5f);

    EXPECT_TRUE((a + b).approx_equal_to(Vec3(-3.0f, 7.0f, 3.5f)));
    EXPECT_TRUE((a - b).approx_equal_to(Vec3(5.0f, -3.0f, 2.5f)));
    EXPECT_TRUE(a.scale(2.0f).approx_equal_to(Vec3(2.0f, 4.0f, 6.0f)));

    EXPECT_FLOAT_EQ(a.dot(b), 1.0f * -4.0f + 2.0f * 5.0f + 3.0f * 0.5f);

    const Vec3 cx = Vec3::unit_x().cross(Vec3::unit_y());
    EXPECT_TRUE(cx.approx_equal_to(Vec3::unit_z()));

    EXPECT_FLOAT_EQ(Vec3(3.0f, 4.0f, 0.0f).length(), 5.0f);
    EXPECT_TRUE(Vec3(3.0f, 4.0f, 0.0f).normalized().approx_equal_to(Vec3(0.6f, 0.8f, 0.0f)));
    // Degenerate normalization must not produce NaN.
    EXPECT_TRUE(Vec3::zero().normalized().approx_equal_to(Vec3::zero()));
}

TEST(Math, QuatRotationMatchesMatrixRotation) {
    const std::vector<Vec3> points = {
        Vec3(1.0f, 0.0f, 0.0f),
        Vec3(0.0f, 1.0f, 0.0f),
        Vec3(0.0f, 0.0f, 1.0f),
        Vec3(2.0f, -3.0f, 0.75f),
        Vec3(-1.5f, 0.25f, 4.0f),
    };

    for (const auto& rc : rotation_cases()) {
        const Mat4 m = Mat4::from_quat(rc.q);
        for (const Vec3& p : points) {
            const Vec3 by_quat = rc.q.rotate_vector(p);
            const Vec3 by_mat = m.transform_direction(p);
            EXPECT_NEAR(by_quat.x, by_mat.x, 1.0e-5f) << rc.name;
            EXPECT_NEAR(by_quat.y, by_mat.y, 1.0e-5f) << rc.name;
            EXPECT_NEAR(by_quat.z, by_mat.z, 1.0e-5f) << rc.name;
        }
    }
}

TEST(Math, RotationMatrixIsOrthonormal) {
    for (const auto& rc : rotation_cases()) {
        const Mat4 m = Mat4::from_quat(rc.q);

        const Vec3 c0(m.at(0, 0), m.at(1, 0), m.at(2, 0));
        const Vec3 c1(m.at(0, 1), m.at(1, 1), m.at(2, 1));
        const Vec3 c2(m.at(0, 2), m.at(1, 2), m.at(2, 2));

        EXPECT_NEAR(c0.length(), 1.0f, 1.0e-5f) << rc.name;
        EXPECT_NEAR(c1.length(), 1.0f, 1.0e-5f) << rc.name;
        EXPECT_NEAR(c2.length(), 1.0f, 1.0e-5f) << rc.name;

        EXPECT_NEAR(c0.dot(c1), 0.0f, 1.0e-5f) << rc.name;
        EXPECT_NEAR(c0.dot(c2), 0.0f, 1.0e-5f) << rc.name;
        EXPECT_NEAR(c1.dot(c2), 0.0f, 1.0e-5f) << rc.name;

        // R * R^T == I.
        const Mat4 ident = m * m.transposed();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                EXPECT_NEAR(ident.at(r, c), r == c ? 1.0f : 0.0f, 1.0e-5f) << rc.name;
            }
        }
    }
}

TEST(Math, FromEulerMatchesSequentialAxisRotations) {
    const float rx = deg_to_rad(25.0f);
    const float ry = deg_to_rad(-40.0f);
    const float rz = deg_to_rad(110.0f);

    const Quat euler = Quat::from_euler(rx, ry, rz);
    const Quat manual = Quat::from_axis_angle(Vec3::unit_z(), rz) *
                        Quat::from_axis_angle(Vec3::unit_y(), ry) *
                        Quat::from_axis_angle(Vec3::unit_x(), rx);

    EXPECT_TRUE(euler.approx_equal_to(manual, 1.0e-5f));
}

TEST(Math, SlerpEndpoints) {
    const Quat a = Quat::from_axis_angle(Vec3(1.0f, 1.0f, 0.0f), deg_to_rad(15.0f));
    const Quat b = Quat::from_axis_angle(Vec3(0.0f, 1.0f, 1.0f), deg_to_rad(170.0f));

    EXPECT_TRUE(slerp(a, b, 0.0f).approx_equal_to(a, 1.0e-5f));
    EXPECT_TRUE(slerp(a, b, 1.0f).approx_equal_to(b, 1.0e-5f));
}

TEST(Math, SlerpConstantAngularVelocityMidpoint) {
    // Halfway between identity and a 90 degree yaw must be a 45 degree yaw.
    const Quat a = Quat::identity();
    const Quat b = Quat::from_axis_angle(Vec3::unit_z(), 90.0f * kDegToRad);
    const Quat mid = slerp(a, b, 0.5f);
    const Quat expected = Quat::from_axis_angle(Vec3::unit_z(), 45.0f * kDegToRad);
    EXPECT_TRUE(mid.approx_equal_to(expected, 1.0e-5f));

    // The rotated-Z direction should have equal angular spacing at each step.
    const int steps = 4;
    float prev_angle = 0.0f;
    for (int i = 1; i <= steps; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(steps);
        const Vec3 v = slerp(a, b, t).rotate_vector(Vec3::unit_y());
        // Rotating +Y about +Z by theta yields (-sin theta, cos theta, 0).
        const float angle = -std::atan2(v.x, v.y);  // yaw about Z
        const float delta = angle - prev_angle;
        EXPECT_NEAR(delta, kHalfPi / static_cast<float>(steps), 1.0e-4f);
        prev_angle = angle;
    }
    EXPECT_NEAR(prev_angle, kHalfPi, 1.0e-4f);
}

TEST(Math, SlerpShortestArcAndLerpFallback) {
    // b negated represents the same rotation; shortest arc must still return
    // something close to b at t=1 and follow the short path in between.
    const Quat a = Quat::from_axis_angle(Vec3::unit_x(), deg_to_rad(10.0f));
    const Quat b = Quat::from_axis_angle(Vec3::unit_x(), deg_to_rad(80.0f));
    const Quat b_neg(-b.w, -b.x, -b.y, -b.z);

    EXPECT_TRUE(slerp(a, b_neg, 1.0f).approx_equal_to(b, 1.0e-5f));

    const Vec3 mid_short = slerp(a, b, 0.5f).rotate_vector(Vec3::unit_y());
    const Vec3 mid_neg = slerp(a, b_neg, 0.5f).rotate_vector(Vec3::unit_y());
    EXPECT_TRUE(mid_short.approx_equal_to(mid_neg, 1.0e-5f));

    // Near-parallel inputs exercise the lerp fallback and must stay unit length
    // and land exactly on the endpoint.
    const Quat c = Quat::from_axis_angle(Vec3::unit_z(), deg_to_rad(30.0f));
    const Quat d = Quat::from_axis_angle(Vec3::unit_z(), deg_to_rad(30.00005f));
    const Quat near = slerp(c, d, 0.37f);
    EXPECT_NEAR(near.length(), 1.0f, 1.0e-5f);
    EXPECT_TRUE(slerp(c, d, 1.0f).approx_equal_to(d, 1.0e-5f));
}

TEST(Math, InverseAffineRoundTrip) {
    const Quat q = Quat::from_euler(deg_to_rad(12.0f), deg_to_rad(88.0f), deg_to_rad(-33.0f));
    const Mat4 trs = Mat4::from_trs(Vec3(2.0f, -5.0f, 7.5f), q, Vec3(1.5f, 2.0f, 0.75f));
    const Mat4 trs_inv = trs.inverse_affine();

    const Mat4 identity_prod = trs * trs_inv;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(identity_prod.at(r, c), r == c ? 1.0f : 0.0f, 1.0e-5f);
        }
    }

    // Points survive a forward/inverse round trip.
    const std::vector<Vec3> pts = {Vec3(0, 0, 0), Vec3(1, 2, 3), Vec3(-4, 0.5f, 9)};
    for (const Vec3& p : pts) {
        const Vec3 rt = trs_inv.transform_point(trs.transform_point(p));
        EXPECT_TRUE(rt.approx_equal_to(p, 1.0e-4f));
    }

    // Translation-only and scale-only inverses behave too.
    const Mat4 t = Mat4::from_translation(Vec3(-1.0f, 4.0f, 2.0f));
    EXPECT_TRUE(t.inverse_affine().transform_point(t.transform_point(Vec3(3, 3, 3)))
                    .approx_equal_to(Vec3(3, 3, 3), 1.0e-5f));

    const Mat4 s = Mat4::from_scale(Vec3(2.0f, 0.5f, 4.0f));
    EXPECT_TRUE(s.inverse_affine().transform_point(s.transform_point(Vec3(1, 1, 1)))
                    .approx_equal_to(Vec3(1, 1, 1), 1.0e-5f));
}

TEST(Math, CompositionIsNonCommutative) {
    const Mat4 t = Mat4::from_translation(Vec3(5.0f, 0.0f, 0.0f));
    const Mat4 r = Mat4::from_quat(Quat::from_axis_angle(Vec3::unit_z(), kHalfPi));

    const Vec3 p(1.0f, 0.0f, 0.0f);
    const Vec3 tr = (t * r).transform_point(p);  // rotate then translate
    const Vec3 rt = (r * t).transform_point(p);  // translate then rotate

    EXPECT_TRUE(tr.approx_equal_to(Vec3(5.0f, 1.0f, 0.0f), 1.0e-5f));
    EXPECT_TRUE(rt.approx_equal_to(Vec3(0.0f, 6.0f, 0.0f), 1.0e-5f));
    EXPECT_FALSE(tr.approx_equal_to(rt, 1.0e-5f));
}

TEST(Math, ComposeDecomposeTrs) {
    const Vec3 t(1.5f, -2.25f, 6.0f);
    const Quat r = Quat::from_euler(deg_to_rad(35.0f), deg_to_rad(-10.0f), deg_to_rad(120.0f));
    const Vec3 s(2.0f, 3.0f, 0.5f);

    const Mat4 m = Mat4::from_trs(t, r, s);
    Vec3 dt;
    Quat dr;
    Vec3 ds;
    Mat4::decompose(m, dt, dr, ds);

    EXPECT_TRUE(dt.approx_equal_to(t, 1.0e-5f));
    EXPECT_TRUE(dr.approx_equal_to(r, 1.0e-4f));
    EXPECT_TRUE(ds.approx_equal_to(s, 1.0e-5f));

    // Recomposing reproduces the same matrix.
    const Mat4 rebuilt = Mat4::from_trs(dt, dr, ds);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(rebuilt.at(row, col), m.at(row, col), 1.0e-4f);
        }
    }
}

TEST(Math, CameraProjectionAndView) {
    const Mat4 view = Mat4::look_at(Vec3(0.0f, 0.0f, 10.0f), Vec3::zero(), Vec3::unit_y());

    // The origin sits 10 units in front of the camera, i.e. at view-space -Z.
    const Vec3 origin_view = view.transform_point(Vec3::zero());
    EXPECT_TRUE(origin_view.approx_equal_to(Vec3(0.0f, 0.0f, -10.0f), 1.0e-4f));

    const float aspect = 16.0f / 9.0f;
    const Mat4 proj = Mat4::perspective(deg_to_rad(60.0f), aspect, 0.1f, 100.0f);
    const Mat4 view_proj = proj * view;

    const auto clip_w = [&](const Vec3& p) {
        return view_proj.at(3, 0) * p.x + view_proj.at(3, 1) * p.y + view_proj.at(3, 2) * p.z +
               view_proj.at(3, 3);
    };

    // A point on the view axis projects to the centre of the screen.
    EXPECT_NEAR(clip_w(Vec3::zero()), 10.0f, 1.0e-3f);
    const Vec3 ndc = Vec3(view_proj.transform_point(Vec3::zero()).x / clip_w(Vec3::zero()),
                          view_proj.transform_point(Vec3::zero()).y / clip_w(Vec3::zero()),
                          0.0f);
    EXPECT_NEAR(ndc.x, 0.0f, 1.0e-4f);
    EXPECT_NEAR(ndc.y, 0.0f, 1.0e-4f);

    // A point behind the camera has negative w and is culled.
    EXPECT_LT(clip_w(Vec3(0.0f, 0.0f, 20.0f)), 0.0f);

    // A point at the right edge of the frustum lands on ndc.x == 1.
    const float half_width = std::tan(deg_to_rad(30.0f)) * aspect * 10.0f;
    const Vec3 edge(half_width, 0.0f, 0.0f);
    const float w_edge = clip_w(edge);
    const Vec3 edge_ndc = view_proj.transform_point(edge) / w_edge;
    EXPECT_NEAR(edge_ndc.x, 1.0f, 1.0e-3f);
}

TEST(Math, QuatNormalizeAndConjugate) {
    const Quat q(2.0f, 0.0f, 0.0f, 0.0f);
    const Quat n = q.normalize();
    EXPECT_NEAR(n.length(), 1.0f, 1.0e-6f);
    EXPECT_TRUE(n.approx_equal_to(Quat::identity()));

    const Quat r = Quat::from_axis_angle(Vec3::unit_y(), deg_to_rad(60.0f));
    const Vec3 v(1.0f, 2.0f, 3.0f);
    // q^-1 for a unit quaternion is its conjugate.
    const Vec3 back = r.conjugate().rotate_vector(r.rotate_vector(v));
    EXPECT_TRUE(back.approx_equal_to(v, 1.0e-5f));
}
