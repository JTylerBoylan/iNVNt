#pragma once

#include <span>
#include "iNVNt/sim/mujoco_interface.hpp"

namespace nvn::mj
{
    struct ReadSimTime
    {
        SimData *sd = nullptr;
        inline auto operator()() const noexcept
        {
            return sd->time;
        }
    };

    struct ReadBodyPosition
    {
        SimData *sd = nullptr;
        inline const translation3d_t &operator()() const noexcept
        {
            return sd->position;
        }
    };

    struct ReadBodyQuaternion
    {
        SimData *sd = nullptr;
        inline quaternion_t operator()() const noexcept
        {
            return sd->orientation;
        }
    };

    struct ReadBodyLinearVelocity
    {
        SimData *sd = nullptr;
        inline lin_velocity3d_t operator()() const noexcept
        {
            return sd->linear_velocity;
        }
    };

    struct ReadBodyAngularVelocity
    {
        SimData *sd = nullptr;
        inline ang_velocity3d_t operator()() const noexcept
        {
            return sd->angular_velocity;
        }
    };

    struct ReadJointPosition
    {
        SimData *sd = nullptr;
        int act_id;

        explicit ReadJointPosition(SimData *s, int i) : sd(s), act_id(i) {}
        inline radians_t operator()() const noexcept
        {
            return sd->joint_positions[act_id];
        }
    };

    template <int N>
    struct ReadJointPositions
    {
        SimData *sd = nullptr;
        int act_id;

        explicit ReadJointPositions(SimData *s, int i0 = 0) : sd(s), act_id(i0) {}
        inline Eigen::Ref<const vector_t<radians_t, N>> operator()() const noexcept
        {
            return sd->joint_positions.template segment<N>(act_id);
        }
    };

    struct ReadJointVelocity
    {
        SimData *sd = nullptr;
        int act_id;

        explicit ReadJointVelocity(SimData *s, int i) : sd(s), act_id(i) {}
        inline radians_per_second_t operator()() const noexcept
        {
            return sd->joint_velocities[act_id];
        }
    };

    template <int N>
    struct ReadJointVelocities
    {
        SimData *sd = nullptr;
        int act_id;

        explicit ReadJointVelocities(SimData *s, int i0 = 0) : sd(s), act_id(i0) {}
        inline Eigen::Ref<const vector_t<radians_per_second_t, N>> operator()() const noexcept
        {
            return sd->joint_velocities.template segment<N>(act_id);
        }
    };

    struct SetJointTorque
    {
        SimData *sd = nullptr;
        int act_id;

        explicit SetJointTorque(SimData *s, int i) : sd(s), act_id(i) {}
        inline void operator()(newton_meters_t tau) noexcept
        {
            sd->joint_torques[act_id] = tau;
        }
    };

    template <int N>
    struct SetJointTorques
    {
        SimData *sd = nullptr;
        int act_id;

        explicit SetJointTorques(SimData *s, int i0 = 0) : sd(s), act_id(i0) {}
        inline void operator()(const vector_t<newton_meters_t, N> &tau) noexcept
        {
            sd->joint_torques.template segment<N>(act_id) = tau;
        }
    };
}