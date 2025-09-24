#pragma once

#include "iNVNt/core/math.hpp"

namespace nvn
{
    template <typename T, int N>
    struct PIDParams
    {
        scalar_t f;                // update frequency [Hz]
        vector_t<T, N> Kp, Kd, Ki; // gains
    };

    template <typename T, int N, typename ReadP>
    struct PID
    {
        ReadP read_p;
        PIDParams<T, N> params;
        vector_t<T, N> p_err_prev = vector_t<T, N>::Zero();
        vector_t<T, N> p_err_intg = vector_t<T, N>::Zero();

        PID(ReadP p, PIDParams<T, N> params)
            : read_p(std::move(p)), params(std::move(params)) {}

        PID(ReadP p, scalar_t freq,
            const vector_t<T, N> &Kp,
            const vector_t<T, N> &Kd,
            const vector_t<T, N> &Ki)
            : PID(std::move(p), {freq, Kp, Kd, Ki}) {}

        template <typename P>
        inline vector_t<T, N> operator()(const P &p_des)
        {
            const auto p = read_p();
            const auto p_err = p_des - p;
            const auto dp_err = (p_err - p_err_prev) * params.f;

            p_err_prev = p_err;
            p_err_intg += p_err / params.f;

            vector_t<T, N> u;
            u.noalias() = params.Kp.cwiseProduct(p_err) +
                          params.Kd.cwiseProduct(dp_err) +
                          params.Ki.cwiseProduct(p_err_intg);
            return u;
        }
    };

    template <typename ReadP>
    using PID_3D = PID<scalar_t, 3, ReadP>;

    // Scalar specialization (N == 1)
    template <typename T, typename ReadP>
    struct PID<T, 1, ReadP>
    {
        ReadP read_p;
        scalar_t f;
        T Kp, Kd, Ki;
        T p_err_prev = T(0);
        T p_err_intg = T(0);

        PID(ReadP p, scalar_t freq, T Kp_, T Kd_, T Ki_)
            : read_p(std::move(p)),
              f(freq), Kp(Kp_), Kd(Kd_), Ki(Ki_) {}

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