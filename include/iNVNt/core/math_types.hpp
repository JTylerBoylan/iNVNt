#pragma once

#include <array>
#include <functional>
#include "eigen3/Eigen/Dense"

namespace nvn
{
    // types
    using scalar_t = double;
    template <typename T, int N>
    using vector_t = Eigen::Vector<T, N>;
    template <typename T, int M, int N>
    using matrix_t = Eigen::Matrix<scalar_t, M, N>;

    constexpr inline auto Dynamic = Eigen::Dynamic;

    // time
    using seconds_t = scalar_t;

    // position
    using meters_t = scalar_t;
    using radians_t = scalar_t;

    // velocity
    using meters_per_second_t = scalar_t;
    using radians_per_second_t = scalar_t;

    // force
    using newtons_t = scalar_t;
    using newton_meters_t = scalar_t;

    // mass
    using kilogram_t = scalar_t;
    using inertia_t = matrix_t<scalar_t, 3, 3>;

    // kinematics
    template <int M, int N>
    using jacobian_t = matrix_t<scalar_t, M, N>;

    // 3D position
    using translation3d_t = vector_t<meters_t, 3>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;
    using euler_angles_t = vector_t<radians_t, 3>;
    // 3D velocity
    using lin_velocity3d_t = vector_t<meters_per_second_t, 3>;
    using ang_velocity3d_t = vector_t<radians_per_second_t, 3>;

    // 3D kinematics
    using jacobian3d_t = jacobian_t<3, 3>;

    // 3D transforms
    using transform3d_t = Eigen::Transform<scalar_t, 3, Eigen::Isometry>;

    // R3 space index aliases (3D vectors)
    enum R3 : unsigned int
    {
        X,
        Y,
        Z
    };

    // R4 space index aliases (quaternions)
    enum R4 : unsigned int
    {
        QW,
        QX,
        QY,
        QZ
    };

    // R9 space index aliases (3D matrices, row-major)
    enum R9 : unsigned int
    {
        XX,
        XY,
        XZ,
        YX,
        YY,
        YZ,
        ZX,
        ZY,
        ZZ
    };

    using index_t = std::size_t;

    struct IndexPairHash
    {
        std::size_t operator()(const std::pair<index_t, index_t> &p) const
        {
            return std::hash<index_t>{}(p.first) ^ (std::hash<index_t>{}(p.second) << 1);
        }
    };
}