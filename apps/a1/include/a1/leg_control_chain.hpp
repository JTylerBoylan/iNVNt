#pragma once

#include "a1/a1_state.hpp"
#include "iNVNt/control/pid.hpp"

namespace a1
{
    using A1_JacobianMap = JacobianMap<scalar_t, 3, 3, A1_ReadJacobian3D &>;
    using A1_LegPID_Compute = PID<scalar_t, 3, A1_ReadLegPosition>;

    inline auto make_leg_control_chain(A1_State &state, A1_Control &control,
                                       A1_LegID leg_id,
                                       const scalar_t freq,
                                       const vector_t<scalar_t, 3> &Kp,
                                       const vector_t<scalar_t, 3> &Kd,
                                       const vector_t<scalar_t, 3> &Ki)
    {
        // In: force -> Out: torques
        auto jac_map = A1_JacobianMap(state.legs[leg_id].jacobian);
        // In: force -> Out: ()
        auto set_force = Chain(std::move(jac_map), std::ref(control.legs[leg_id].joint_torques));

        // In: setpoint -> Out: force
        auto leg_pid_compute = A1_LegPID_Compute(state.legs[leg_id].position, freq, Kp, Kd, Ki);
        // In: () -> Out: force
        auto leg_pid = Chain(std::ref(control.legs[leg_id].position_setpoint), std::move(leg_pid_compute));

        // In: () -> Out: force
        auto stance_cond = Branch(std::ref(control.legs[leg_id].is_in_stance),
                                  std::ref(control.legs[leg_id].force_setpoint),
                                  std::move(leg_pid));
        
        // In: () -> Out: ()
        auto leg_control = Chain(std::move(stance_cond), std::move(set_force));

        return leg_control;
    }
}