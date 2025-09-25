#pragma once

#include "iNVNt/core/math.hpp"

namespace nvn
{
    enum PIDControlType
    {
        P,
        PD,
        PID
    };

    template <typename T, int N>
    struct PIDParams
    {
        scalar_t f;                // update frequency [Hz]
        vector_t<T, N> Kp, Kd, Ki; // gains
    };

    template <typename T, int N, PIDControlType C>
    struct ComputePIDControl
    {
        static_assert(P <= C && C <= PID, "Invalid PID Control Type");

        PIDParams<T, N> params;
        vector_t<T, N> p_err_prev = vector_t<T, N>::Zero();
        vector_t<T, N> p_err_intg = vector_t<T, N>::Zero();

        ComputePIDControl(const PIDParams<T, N> &params)
            : params(params) {}

        ComputePIDControl(scalar_t freq,
                          const vector_t<T, N> &Kp,
                          const vector_t<T, N> &Kd = vector_t<T, N>::Zero(),
                          const vector_t<T, N> &Ki = vector_t<T, N>::Zero())
            : params({freq, Kp, Kd, Ki}) {}

        template <typename P, typename Q>
        inline vector_t<T, N> operator()(const P &p_des, const Q &p)
        {
            const auto p_err = p_des - p;
            vector_t<T, N> u = params.Kp.cwiseProduct(p_err);
            if constexpr (C == PD || C == PID)
            {
                const auto dp_err = (p_err - p_err_prev) * params.f;
                p_err_prev = p_err;
                u += params.Kd.cwiseProduct(dp_err);
            }
            if constexpr (C == PID)
            {
                p_err_intg += p_err / params.f;
                u += params.Ki.cwiseProduct(p_err_intg);
            }
            return u;
        }
    };

    // Scalar specialization (N == 1)
    template <typename T>
    struct PIDParams<T, 1>
    {
        scalar_t f;   // update frequency [Hz]
        T Kp, Kd, Ki; // gains
    };

    template <typename T, PIDControlType C>
    struct ComputePIDControl<T, 1, C>
    {
        PIDParams<T, 1> params;
        T p_err_prev = T(0);
        T p_err_intg = T(0);

        ComputePIDControl(const PIDParams<T, 1> &params)
            : params(params) {}

        ComputePIDControl(scalar_t freq, T Kp, T Kd = T{}, T Ki = T{})
            : params({freq, Kp, Kd, Ki}) {}

        template <typename P, typename Q>
        inline T operator()(P p_des, Q p)
        {
            const T p_err = p_des - p;
            auto u = params.Kp * p_err;
            if constexpr (C == PD || C == PID)
            {
                const T dp_err = (p_err - p_err_prev) * params.f;
                p_err_prev = p_err;
                u += params.Kd * dp_err;
            }
            if constexpr (C == PID)
            {
                p_err_intg += p_err / params.f;
                u += params.Ki * p_err_intg;
            }
            return u;
        }
    };

    template <typename T, int N, PIDControlType C, typename ReadP>
    struct PIDControl
    {
        ReadP read_p;
        ComputePIDControl<T, N, C> compute_pid;

        PIDControl(ReadP p, const PIDParams<T, N> &params)
            : read_p(std::move(p)), compute_pid(params) {}

        PIDControl(ReadP p, scalar_t freq,
                   const vector_t<T, N> &Kp,
                   const vector_t<T, N> &Kd = vector_t<T, N>::Zero(),
                   const vector_t<T, N> &Ki = vector_t<T, N>::Zero())
            : read_p(std::move(p)), compute_pid(freq, Kp, Kd, Ki) {}

        template <typename P>
        inline vector_t<T, N> operator()(const P &p_des)
        {
            return compute_pid(p_des, read_p());
        }
    };

    template <typename T, PIDControlType C, typename ReadP>
    struct PIDControl<T, 1, C, ReadP>
    {
        ReadP read_p;
        ComputePIDControl<T, 1, C> compute_pid;

        PIDControl(ReadP p, PIDParams<T, 1> params)
            : read_p(std::move(p)), compute_pid(params) {}

        PIDControl(ReadP p, scalar_t freq, const T Kp, const T Kd = T{}, const T Ki = T{})
            : read_p(std::move(p)), compute_pid({freq, Kp, Kd, Ki}) {}

        template <typename P>
        inline T operator()(const P &p_des)
        {
            return compute_pid(p_des, read_p());
        }
    };

    template <typename ReadP>
    using PIDControl3D = PIDControl<scalar_t, 3, PID, ReadP>;
}