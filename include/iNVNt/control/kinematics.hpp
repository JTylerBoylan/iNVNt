#pragma once

#include "iNVNt/core/math.hpp"

namespace nvn
{
    template <typename ReadJ>
    struct JacobianMap
    {
        ReadJ &read_J;

        explicit JacobianMap(ReadJ &jac) : read_J(jac) {}

        template <typename T>
        inline auto operator()(const T &vec)
        {
            return read_J().transpose() * vec;
        }
    };
}