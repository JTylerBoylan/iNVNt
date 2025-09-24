#pragma once

#include "a1/a1_types.hpp"

namespace a1
{
    using A1_SetJointTorque = mj::SetJointTorque;
    using A1_SetAllJointTorques = mj::SetJointTorques<A1_NUM_JOINTS>;
    struct A1_JointControls
    {
        std::array<A1_SetJointTorque, A1_NUM_JOINTS> torques;
        A1_SetAllJointTorques torques_vec;

        A1_JointControls(SimData *sd)
            : torques({A1_SetJointTorque(sd, FR_ABDUCT),
                       A1_SetJointTorque(sd, FR_HIP),
                       A1_SetJointTorque(sd, FR_KNEE),
                       A1_SetJointTorque(sd, FL_ABDUCT),
                       A1_SetJointTorque(sd, FL_HIP),
                       A1_SetJointTorque(sd, FL_KNEE),
                       A1_SetJointTorque(sd, RR_ABDUCT),
                       A1_SetJointTorque(sd, RR_HIP),
                       A1_SetJointTorque(sd, RR_KNEE),
                       A1_SetJointTorque(sd, RL_ABDUCT),
                       A1_SetJointTorque(sd, RL_HIP),
                       A1_SetJointTorque(sd, RL_KNEE)}),
              torques_vec(sd)
        {
        }
    };

    using A1_SetLegJointTorques = mj::SetJointTorques<A1_NUM_JOINTS_PER_LEG>;
    struct A1_LegControl
    {
        A1_SetLegJointTorques joint_torques;

        A1_LegControl(SimData *sd, A1_LegID leg_id)
            : joint_torques(sd, leg_id * A1_NUM_JOINTS_PER_LEG) {}
    };

    struct A1_Control
    {
        A1_JointControls joints;
        std::array<A1_LegControl, A1_NUM_LEGS> legs;

        A1_Control(SimData *sd)
            : joints(sd), legs{A1_LegControl(sd, FR),
                               A1_LegControl(sd, FL),
                               A1_LegControl(sd, RR),
                               A1_LegControl(sd, RL)} {}
    };
};