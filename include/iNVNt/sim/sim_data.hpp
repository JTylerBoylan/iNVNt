#pragma once

#include <vector>

#include "iNVNt/core/math_types.hpp"

namespace nvn
{
    struct SimData
    {
        seconds_t time = 0.0;                                       // simulation time
        translation3d_t position = {0, 0, 0};                       // position in world frame
        quaternion_t orientation = {1, 0, 0, 0};                    // orientation in world frame
        lin_velocity3d_t linear_velocity = {0, 0, 0};               // linear velocity in world frame
        ang_velocity3d_t angular_velocity = {0, 0, 0};              // angular velocity in world frame
        vector_t<radians_t, Dynamic> joint_positions{};             // joint positions
        vector_t<radians_per_second_t, Dynamic> joint_velocities{}; // joint velocities
        vector_t<newton_meters_t, Dynamic> joint_torques{};         // joint torques

        SimData() = default;
        SimData(size_t num_joints)
            : joint_positions(num_joints),
              joint_velocities(num_joints),
              joint_torques(num_joints) {}
    };
}