#pragma once

#include "iNVNt/core/math.hpp"

namespace nvn
{
    template <typename T, int N, typename ReadP>
        requires Reader<ReadP, vector_t<T, N>>
    struct PID
    {
        ReadP &read_p;             // reference, not owned
        scalar_t f;                // update frequency [Hz]
        vector_t<T, N> Kp, Kd, Ki; // gains
        vector_t<T, N> p_err_prev = vector_t<T, N>::Zero();
        vector_t<T, N> p_err_intg = vector_t<T, N>::Zero();

        PID(ReadP &p, scalar_t freq,
            const Eigen::Ref<const vector_t<T, N>> &Kp_,
            const Eigen::Ref<const vector_t<T, N>> &Kd_,
            const Eigen::Ref<const vector_t<T, N>> &Ki_)
            : read_p(p), f(freq), Kp(Kp_), Kd(Kd_), Ki(Ki_) {}

        inline vector_t<T, N> operator()(const vector_t<T, N> &p_des)
        {
            const auto p = read_p();
            const auto p_err = p_des - p;
            const auto dp_err = (p_err - p_err_prev) * f;

            p_err_prev = p_err;
            p_err_intg += p_err / f;

            vector_t<T, N> u;
            u.noalias() = Kp.cwiseProduct(p_err) + Kd.cwiseProduct(dp_err) + Ki.cwiseProduct(p_err_intg);
            return u;
        }
    };

    // Scalar specialization (N == 1)
    template <typename T, typename ReadP>
        requires Reader<ReadP, T>
    struct PID<T, 1, ReadP>
    {
        ReadP &read_p;
        scalar_t f;
        T Kp, Kd, Ki;
        T p_err_prev = T(0);
        T p_err_intg = T(0);

        PID(ReadP &p, scalar_t freq, T Kp_, T Kd_, T Ki_)
            : read_p(p), f(freq), Kp(Kp_), Kd(Kd_), Ki(Ki_) {}

        inline T operator()(T p_des)
        {
            const T p = read_p();
            const T p_err = p_des - p;
            const T dp_err = (p_err - p_err_prev) * f;

            p_err_prev = p_err;
            p_err_intg += p_err / f;

            return Kp * p_err + Kd * dp_err + Ki * p_err_intg;
        }
    };
}