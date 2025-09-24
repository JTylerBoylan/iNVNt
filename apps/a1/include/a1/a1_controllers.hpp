#pragma once

#include "a1/a1_state.hpp"
#include "a1/a1_control.hpp"
#include "a1/param_server.hpp"
#include "iNVNt/control/pid.hpp"

namespace a1
{
    struct A1_ToeForceController
    {
        A1_ReadToeJacobian3D read_J;
        A1_SetLegJointTorques set_tau;

        A1_ToeForceController(const A1_ReadToeJacobian3D &J, const A1_SetLegJointTorques &tau)
            : read_J(J), set_tau(tau) {}

        inline void operator()(const vector_t<newtons_t, 3> &force)
        {
            auto tau = read_J().transpose() * force;
            set_tau(tau);
        }
    };

    using A1_ToePID = PID_3D<A1_ReadToePosition>;
    struct A1_ToePositionController
    {
        A1_ToeForceController ctrl_force;
        A1_ToePID pid;

        A1_ToePositionController(const A1_ToeForceController &cf, const A1_ReadToePosition &p,
                                 const PIDParams<scalar_t, 3> &pid_params)
            : ctrl_force(cf),
              pid(p, pid_params) {}

        inline void operator()(const translation3d_t &pos)
        {
            auto force = pid(pos);
            ctrl_force(force);
        }
    };

    using A1_IsToeInStance = ReadWrite<bool>;
    using A1_LegPositionSetpoint = ReadWrite<translation3d_t>;
    using A1_LegForceSetpoint = ReadWrite<vector_t<newtons_t, 3>>;
    struct A1_ToeController
    {
        A1_IsToeInStance is_in_stance;
        A1_LegForceSetpoint force_setpoint;
        A1_LegPositionSetpoint pos_setpoint;

        A1_ToeForceController ctrl_force;
        A1_ToePositionController ctrl_pos;

        A1_ToeController(const A1_LegState &state, const A1_LegControl &control,
                         const A1_ParameterServer &params)
            : is_in_stance(false),
              force_setpoint({0.0, 0.0, 0.0}),
              pos_setpoint(state.default_toe_position()),
              ctrl_force(state.toe_jacobian, control.joint_torques),
              ctrl_pos(ctrl_force, state.toe_position,
                       {100,
                        vector_t<scalar_t, 3>::Constant(1000.0),
                        vector_t<scalar_t, 3>::Constant(10.0),
                        vector_t<scalar_t, 3>::Constant(1.0)}) {}

        inline void operator()()
        {
            if (is_in_stance())
            {
                ctrl_force(force_setpoint());
            }
            else
            {
                ctrl_pos(pos_setpoint());
            }
        }
    };

    struct A1_Controllers
    {
        std::array<A1_ToeController, A1_NUM_LEGS> toe_ctrl;

        A1_Controllers(const A1_State &state, const A1_Control &control,
                       const A1_ParameterServer &params)
            : toe_ctrl({A1_ToeController(state.legs[FR], control.legs[FR], params),
                        A1_ToeController(state.legs[FL], control.legs[FL], params),
                        A1_ToeController(state.legs[RR], control.legs[RR], params),
                        A1_ToeController(state.legs[RL], control.legs[RL], params)}) {}
    };
}