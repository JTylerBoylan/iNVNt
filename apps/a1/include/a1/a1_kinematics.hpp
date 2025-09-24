#pragma once

#include <math.h>
#include "iNVNt/control/kinematics.hpp"

namespace a1
{
    using namespace nvn;

    struct ComputeForwardKinematics
    {
        scalar_t ax = 1.0; // +1 for right leg, -1 for left leg
        meters_t ls, lt, lc;

        ComputeForwardKinematics(bool is_right_leg = true,
                                 meters_t shoulder_link_length = 0.08505,
                                 meters_t thigh_link_length = 0.2,
                                 meters_t calf_link_length = 0.2)
            : ax(is_right_leg ? -1.0 : +1.0),
              ls(shoulder_link_length), lt(thigh_link_length), lc(calf_link_length) {}

        inline translation3d_t operator()(const vector_t<radians_t, 3> &q) const noexcept
        {
            const float q1 = q[0];
            const float q2 = q[1];
            const float q3 = q[2];

            const float s1 = std::sin(q1);
            const float c1 = std::cos(q1);
            const float s2 = std::sin(q2);
            const float c2 = std::cos(q2);
            const float s3 = std::sin(q3);
            const float c3 = std::cos(q3);
            const float s23 = std::sin(q2 + q3);

            const float x = -lc * s23 - lt * s2;
            const float y = ax * ls * c1 - lc * (s1 * s2 * s3 - c2 * c3 * s1) + lt * c2 * q1;
            const float z = ax * ls * s1 - lc * (c1 * c2 * c3 - c1 * s2 * s3) - lt * c1 * c2;

            return {x, y, z};
        }
    };

    struct ComputeInverseKinematics
    {
        scalar_t ax = 1.0; // +1 for right leg, -1 for left leg
        meters_t ls, lt, lc;

        ComputeInverseKinematics(bool is_right_leg = true,
                                 meters_t shoulder_link_length = 0.08505,
                                 meters_t thigh_link_length = 0.2,
                                 meters_t calf_link_length = 0.2)
            : ax(is_right_leg ? -1.0 : +1.0),
              ls(shoulder_link_length), lt(thigh_link_length), lc(calf_link_length) {}
        inline vector_t<radians_t, 3> operator()(const translation3d_t &position) const noexcept
        {
            const float x = position[0];
            const float y = ax * position[1];
            const float z = position[2];
            // Get q1 from Y-Z plane
            const float r1 = std::sqrt(y * y + z * z);
            const float alpha = std::asin(ls / r1);
            const float beta = M_PI_2 - alpha;
            const float gamma = std::atan2(z, y);
            const float q1 = beta + gamma;
            // Transform to the new coordinate system
            const float dy = y - ls * std::cos(q1);
            const float dz = z - ls * std::sin(q1);
            const float zp = -dy * sin(q1) + dz * cos(q1);
            const float r2 = std::sqrt(x * x + zp * zp);
            const float phi = std::atan2(zp, x);
            const float f = phi + M_PI_2;
            const float lambda = std::acos((r2 * r2 + lt * lt - lc * lc) / (2 * r2 * lt));
            const float del = std::asin(lt * std::sin(lambda) / lc);
            const float epsilon = M_PI - del - lambda;
            const float q2 = f - lambda;
            const float q3 = M_PI - epsilon;
            return {q1 * ax, -q2, -q3};
        }
    };

    struct ComputeJacobian
    {
        scalar_t ax = 1.0; // +1 for right leg, -1 for left leg
        meters_t ls, lt, lc;

        explicit ComputeJacobian(bool is_right_leg = true,
                                 meters_t shoulder_link_length = 0.08505,
                                 meters_t thigh_link_length = 0.2,
                                 meters_t calf_link_length = 0.2)
            : ax(is_right_leg ? -1.0 : +1.0),
              ls(shoulder_link_length), lt(thigh_link_length), lc(calf_link_length) {}

        inline jacobian3d_t operator()(const vector_t<radians_t, 3> &q) const noexcept
        {
            const radians_t abduction = q[0];
            const radians_t hip = q[1];
            const radians_t knee = q[2];

            const scalar_t s1 = std::sin(abduction);
            const scalar_t c1 = std::cos(abduction);
            const scalar_t s2 = std::sin(hip);
            const scalar_t c2 = std::cos(hip);
            const scalar_t s3 = std::sin(knee);
            const scalar_t c3 = std::cos(knee);
            const scalar_t s23 = std::sin(hip + knee);
            const scalar_t c23 = std::cos(hip + knee);

            const scalar_t dXdq1 = 0.0;
            const scalar_t dXdq2 = -lc * c23 - lt * c2;
            const scalar_t dXdq3 = -lc * c23;
            const scalar_t dYdq1 = lc * (c1 * c2 * c3 - c1 * s2 * s3) - ax * ls * s1 + lt * c1 * c2;
            const scalar_t dYdq2 = -s1 * (lc * s23 + lt * s2);
            const scalar_t dYdq3 = -lc * s1 * s23;
            const scalar_t dZdq1 = ax * ls * c1 - lc * (s1 * s2 * s3 - c2 * c3 * s1) + lt * c2 * s1;
            const scalar_t dZdq2 = c1 * (lc * s23 + lt * s2);
            const scalar_t dZdq3 = lc * c1 * s23;

            jacobian3d_t jac;
            jac << dXdq1, dXdq2, dXdq3,
                dYdq1, dYdq2, dYdq3,
                dZdq1, dZdq2, dZdq3;
            return jac;
        }
    };

    template <typename ReadQ>
    struct ReadToePosition
    {
        ReadQ read_q;
        ComputeForwardKinematics compute_fk;

        ReadToePosition(ReadQ q, bool is_right_leg = true,
                        meters_t shoulder_link_length = 0.08505,
                        meters_t thigh_link_length = 0.2,
                        meters_t calf_link_length = 0.2)
            : read_q(q),
              compute_fk(is_right_leg, shoulder_link_length, thigh_link_length, calf_link_length) {}

        inline translation3d_t operator()() const noexcept
        {
            return compute_fk(read_q());
        }
    };

    template <typename ReadQ>
    struct ReadToeJacobian
    {
        ReadQ read_q;
        ComputeJacobian compute_jac;

        explicit ReadToeJacobian(ReadQ q,
                                 bool is_right_leg = true,
                                 meters_t shoulder_link_length = 0.08505,
                                 meters_t thigh_link_length = 0.2,
                                 meters_t calf_link_length = 0.2)
            : read_q(q),
              compute_jac(is_right_leg, shoulder_link_length, thigh_link_length, calf_link_length) {}

        inline jacobian3d_t operator()() const
        {
            return compute_jac(read_q());
        }
    };
}