#pragma once

#include "iNVNt/core/math.hpp"

namespace nvn
{
    template <typename T, int M, int N, typename ReadJ>
        requires JacobianReader<ReadJ, M, N>
    struct JacobianMap
    {
        ReadJ read_J;

        template <typename RJ>
        explicit JacobianMap(RJ &&jac)
            : read_J(std::forward<RJ>(jac)) {}

        inline auto operator()(const Eigen::Ref<const vector_t<T, M>> &vec) const
            -> vector_t<T, N>
        {
            vector_t<T, N> out;
            out.noalias() = read_J().transpose() * vec;
            return out;
        }
    };

    template <typename T, int M, int N, typename ReadJ>
    JacobianMap(ReadJ &&J) -> JacobianMap<T, M, N, ReadJ>;

    template <typename T, int M, int N, JacobianReader<M, N> ReadJ, VectorWriter<T, N> SetT, typename... Args>
    using SetForce = Chain<JacobianMap<T, M, N, ReadJ>, SetT>;
}