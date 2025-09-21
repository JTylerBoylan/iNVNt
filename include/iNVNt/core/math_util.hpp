#pragma once

#include "iNVNt/core/math_concepts.hpp"

namespace nvn
{
    template <JointStateReader ReadQ>
    struct ReadJointQuaternion
    {
        using Q = std::unwrap_ref_decay_t<ReadQ>;
        Q read_q;

        template <typename QQ>
        ReadJointQuaternion(QQ &&q)
            : read_q(std::forward<QQ>(q)) {}

        inline quaternion_t operator()() const noexcept
        {
            return quaternion_t(Eigen::AngleAxis<scalar_t>(read_q(), vector_t<scalar_t, 3>::UnitZ()));
        }
    };

    struct IdentityQuaternion
    {
        inline quaternion_t operator()() const { return quaternion_t::Identity(); }
    };

    struct IdentityTranslation3D
    {
        inline translation3d_t operator()() const { return translation3d_t::Zero(); }
    };

    enum MatrixStorage
    {
        RowMajor,
        ColMajor
    };

    template <int M, int N, MatrixStorage MS = RowMajor>
    inline int flatten(int row, int col) noexcept
    {
        if constexpr (MS == RowMajor)
        {
            return row + M * col;
        }
        else
        {
            return col + N * row;
        }
    }

    template <int M, int N, MatrixStorage MS = RowMajor>
    inline std::pair<int, int> unflatten(int idx) noexcept
    {
        if constexpr (MS == RowMajor)
        {
            return {idx % M, idx / M}; // haven't checked this
        }
        else
        {
            return {idx / N, idx % N};
        }
    }

    template <typename T, int M, int N>
    struct IsMatrixInvertible
    {
        bool operator()(const matrix_t<T, M, N> &mat)
        {
            if constexpr (M != N)
                return false;
            if (mat.determinant() == 0)
                return false;
            return true;
        }
    };
}