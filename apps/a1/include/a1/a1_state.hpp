#pragma once

#include "a1/a1_types.hpp"
#include "a1/a1_kinematics.hpp"

namespace a1
{
    using A1_ReadBodyPosition = mj::ReadBodyPosition;
    using A1_ReadBodyOrientation = mj::ReadBodyQuaternion;
    using A1_ReadBodyLinearVelocity = mj::ReadBodyLinearVelocity;
    using A1_ReadBodyAngularVelocity = mj::ReadBodyAngularVelocity;
    struct A1_Odometry
    {
        A1_ReadBodyPosition position;
        A1_ReadBodyOrientation orientation;
        A1_ReadBodyLinearVelocity lin_velocity;
        A1_ReadBodyAngularVelocity ang_velocity;

        A1_Odometry(SimData *sd)
            : position(sd), orientation(sd), lin_velocity(sd), ang_velocity(sd) {}
    };

    using A1_ReadJointPosition = mj::ReadJointPosition;
    using A1_ReadJointVelocity = mj::ReadJointVelocity;
    using A1_ReadAllJointPositions = mj::ReadJointPositions<A1_NUM_JOINTS>;
    using A1_ReadAllJointVelocities = mj::ReadJointVelocities<A1_NUM_JOINTS>;
    struct A1_JointStates
    {
        std::array<A1_ReadJointPosition, A1_NUM_JOINTS> positions;
        std::array<A1_ReadJointVelocity, A1_NUM_JOINTS> velocities;
        A1_ReadAllJointPositions positions_vec;
        A1_ReadAllJointVelocities velocities_vec;

        A1_JointStates(SimData *sd)
            : positions({A1_ReadJointPosition(sd, FR_ABDUCT),
                         A1_ReadJointPosition(sd, FR_HIP),
                         A1_ReadJointPosition(sd, FR_KNEE),
                         A1_ReadJointPosition(sd, FL_ABDUCT),
                         A1_ReadJointPosition(sd, FL_HIP),
                         A1_ReadJointPosition(sd, FL_KNEE),
                         A1_ReadJointPosition(sd, RR_ABDUCT),
                         A1_ReadJointPosition(sd, RR_HIP),
                         A1_ReadJointPosition(sd, RR_KNEE),
                         A1_ReadJointPosition(sd, RL_ABDUCT),
                         A1_ReadJointPosition(sd, RL_HIP),
                         A1_ReadJointPosition(sd, RL_KNEE)}),
              velocities({A1_ReadJointVelocity(sd, FR_ABDUCT),
                          A1_ReadJointVelocity(sd, FR_HIP),
                          A1_ReadJointVelocity(sd, FR_KNEE),
                          A1_ReadJointVelocity(sd, FL_ABDUCT),
                          A1_ReadJointVelocity(sd, FL_HIP),
                          A1_ReadJointVelocity(sd, FL_KNEE),
                          A1_ReadJointVelocity(sd, RR_ABDUCT),
                          A1_ReadJointVelocity(sd, RR_HIP),
                          A1_ReadJointVelocity(sd, RR_KNEE),
                          A1_ReadJointVelocity(sd, RL_ABDUCT),
                          A1_ReadJointVelocity(sd, RL_HIP),
                          A1_ReadJointVelocity(sd, RL_KNEE)}),
              positions_vec(sd), velocities_vec(sd) {}
    };

    using A1_ReadLegJointPositions = mj::ReadJointPositions<A1_NUM_JOINTS_PER_LEG>;
    using A1_ReadLegJointVelocities = mj::ReadJointVelocities<A1_NUM_JOINTS_PER_LEG>;
    using A1_ReadToePosition = a1::ReadToePosition<A1_ReadLegJointPositions&>;
    using A1_ReadToeJacobian3D = a1::ReadToeJacobian<A1_ReadLegJointPositions&>;
    using A1_DefaultToePosition = ReadWrite<translation3d_t>;
    struct A1_LegState
    {
        A1_ReadLegJointPositions joint_positions;
        A1_ReadLegJointVelocities joint_velocities;
        A1_ReadToePosition toe_position;
        A1_ReadToeJacobian3D toe_jacobian;
        A1_DefaultToePosition default_toe_position;

        A1_LegState(SimData *sd, A1_LegID leg_id,
                    bool is_rl = true, meters_t ls = 0.08505, meters_t lt = 0.2, meters_t lc = 0.2)
            : joint_positions(sd, leg_id * A1_NUM_JOINTS_PER_LEG),
              joint_velocities(sd, leg_id * A1_NUM_JOINTS_PER_LEG),
              toe_position(joint_positions, is_rl, ls, lt, lc),
              toe_jacobian(joint_positions, is_rl, ls, lt, lc),
              default_toe_position({0.0, (is_rl ? -1.0 : +1.0) * A1_DEFAULT_YOFF, -A1_DEFAULT_HEIGHT}) {}
    };

    struct A1_State
    {
        A1_Odometry odom;
        A1_JointStates joints;
        std::array<A1_LegState, A1_NUM_LEGS> legs;

        A1_State(SimData *sd)
            : odom(sd), joints(sd),
              legs{A1_LegState(sd, FR, A1_RIGHT, A1_LS, A1_LT, A1_LC),
                   A1_LegState(sd, FL, A1_LEFT, A1_LS, A1_LT, A1_LC),
                   A1_LegState(sd, RR, A1_RIGHT, A1_LS, A1_LT, A1_LC),
                   A1_LegState(sd, RL, A1_LEFT, A1_LS, A1_LT, A1_LC)} {}
    };
}