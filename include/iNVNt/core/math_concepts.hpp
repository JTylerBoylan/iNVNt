#pragma once

#include "iNVNt/core/func_concepts.hpp"
#include "iNVNt/core/math_types.hpp"

namespace nvn
{
    template <typename F, typename T, int N>
    concept VectorReader = Reader<F, vector_t<T, N>>;

    template <typename F, typename T, int N>
    concept VectorWriter = Writer<F, vector_t<T, N>>;

    template <typename F>
    concept TimeReader = Reader<F, seconds_t>;

    template <typename F>
    concept JointStateReader = Reader<F, radians_t>;

    template <typename F>
    concept Translation3DReader = Reader<F, translation3d_t>;

    template <typename F>
    concept QuaternionReader = Reader<F, quaternion_t>;
    
    template <typename F>
    concept LinearVelocity3DReader = Reader<F, lin_velocity3d_t>;

    template <typename F>
    concept AngularVelocity3DReader = Reader<F, ang_velocity3d_t>;

    template <typename F, int M, int N>
    concept JacobianReader = Reader<F, jacobian_t<M, N>>;
}