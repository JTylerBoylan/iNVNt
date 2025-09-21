#include "a1/unitree_a1.hpp"

/*
    static_assert(can_invoke<A1_IsLegInStance, void>);
    static_assert(can_invoke<A1_LegForceSetpoint, void>);
    static_assert(can_invoke<A1_LegPID_Chain, void>);
    static_assert(std::convertible_to<result_t<A1_IsLegInStance, void>, bool>);
    static_assert(std::same_as<result_t<A1_LegForceSetpoint, void>, result_t<A1_LegPID_Chain, void>>);
    using type1 = result_t<A1_LegForceSetpoint, void>;
    using type2 = result_t<A1_LegPID_Chain, void>;
*/

using namespace nvn;
using namespace unitree_a1;
int main(void)
{
    auto log = std::make_shared<ConsoleLog>();
    *log << "Starting Unitree A1...\n";

    auto a1 = std::make_shared<UnitreeA1>();

    vector_t<3> Kp = {1000.0, 1000.0, 1000.0},
                Kd = {1.0, 1.0, 1.0},
                Ki = {0.0, 0.0, 0.0}; // x y z

    using A1_LegPID_Compute = PIDArray<3, A1_ReadLegPosition>;
    auto pid_compute_fr = make_ref<A1_LegPID_Compute>(Kp, Kd, Ki, a1->state.front_right.toe_position);
    auto pid_compute_fl = make_ref<A1_LegPID_Compute>(Kp, Kd, Ki, a1->state.front_left.toe_position);
    auto pid_compute_rr = make_ref<A1_LegPID_Compute>(Kp, Kd, Ki, a1->state.rear_right.toe_position);
    auto pid_compute_rl = make_ref<A1_LegPID_Compute>(Kp, Kd, Ki, a1->state.rear_left.toe_position);

    using A1_LegPID_Chain = Chain<A1_LegPositionSetpoint, A1_LegPID_Compute>;
    auto pid_chain_fr = make_ref<A1_LegPID_Chain>(a1->control.front_right.toe_position_setpoint, pid_compute_fr);
    auto pid_chain_fl = make_ref<A1_LegPID_Chain>(a1->control.front_left.toe_position_setpoint, pid_compute_fl);
    auto pid_chain_rr = make_ref<A1_LegPID_Chain>(a1->control.rear_right.toe_position_setpoint, pid_compute_rr);
    auto pid_chain_rl = make_ref<A1_LegPID_Chain>(a1->control.rear_left.toe_position_setpoint, pid_compute_rl);

    using A1_LegForce_Compute = Branch<A1_IsLegInStance, A1_LegForceSetpoint, A1_LegPID_Chain>;
    auto force_compute_fr = make_ref<A1_LegForce_Compute>(
        a1->control.front_right.is_in_stance, a1->control.front_right.toe_force_setpoint, pid_chain_fr);
    auto force_compute_fl = make_ref<A1_LegForce_Compute>(
        a1->control.front_left.is_in_stance, a1->control.front_left.toe_force_setpoint, pid_chain_fl);
    auto force_compute_rr = make_ref<A1_LegForce_Compute>(
        a1->control.rear_right.is_in_stance, a1->control.rear_right.toe_force_setpoint, pid_chain_rr);
    auto force_compute_rl = make_ref<A1_LegForce_Compute>(
        a1->control.rear_left.is_in_stance, a1->control.rear_left.toe_force_setpoint, pid_chain_rl);

    using A1_LegTorque_Convert = ConvertForceToTorques<3, A1_ReadJacobian3D>;
    auto force_to_torque_fr = make_ref<A1_LegTorque_Convert>(a1->state.front_right.jacobian_ee);
    auto force_to_torque_fl = make_ref<A1_LegTorque_Convert>(a1->state.front_left.jacobian_ee);
    auto force_to_torque_rr = make_ref<A1_LegTorque_Convert>(a1->state.rear_right.jacobian_ee);
    auto force_to_torque_rl = make_ref<A1_LegTorque_Convert>(a1->state.rear_left.jacobian_ee);

    using A1_LegTorque_Compute = Chain<A1_LegForce_Compute, A1_LegTorque_Convert>;
    auto torque_compute_fr = make_ref<A1_LegTorque_Compute>(force_compute_fr, force_to_torque_fr);
    auto torque_compute_fl = make_ref<A1_LegTorque_Compute>(force_compute_fl, force_to_torque_fl);
    auto torque_compute_rr = make_ref<A1_LegTorque_Compute>(force_compute_rr, force_to_torque_rr);
    auto torque_compute_rl = make_ref<A1_LegTorque_Compute>(force_compute_rl, force_to_torque_rl);

    using A1_LegTorque_Control = Chain<A1_LegTorque_Compute, A1_SetLegJointTorques>;
    auto torque_control_fr = make_ref<A1_LegTorque_Control>(torque_compute_fr, a1->control.front_right.set_torques);
    auto torque_control_fl = make_ref<A1_LegTorque_Control>(torque_compute_fl, a1->control.front_left.set_torques);
    auto torque_control_rr = make_ref<A1_LegTorque_Control>(torque_compute_rr, a1->control.rear_right.set_torques);
    auto torque_control_rl = make_ref<A1_LegTorque_Control>(torque_compute_rl, a1->control.rear_left.set_torques);

    using A1_LegJacobian_Check = Chain<A1_ReadJacobian3D, IsMatrixInvertible<3, 3>>;
    auto check_inv = make_ref<IsMatrixInvertible<3, 3>>();
    auto jac_check_fr = make_ref<A1_LegJacobian_Check>(a1->state.front_right.jacobian_ee, check_inv);
    auto jac_check_fl = make_ref<A1_LegJacobian_Check>(a1->state.front_left.jacobian_ee, check_inv);
    auto jac_check_rr = make_ref<A1_LegJacobian_Check>(a1->state.rear_right.jacobian_ee, check_inv);
    auto jac_check_rl = make_ref<A1_LegJacobian_Check>(a1->state.rear_left.jacobian_ee, check_inv);

    using A1_LegControl = Branch<A1_LegJacobian_Check, A1_LegTorque_Control, LogWriteConst<PrintConsole>>;
    auto print_error = make_ref<LogWriteConst<PrintConsole>>(a1->log, "Jacobian not invertible!\n");
    auto leg_control_fr = make_ref<A1_LegControl>(jac_check_fr, torque_control_fr, print_error);
    auto leg_control_fl = make_ref<A1_LegControl>(jac_check_fl, torque_control_fl, print_error);
    auto leg_control_rr = make_ref<A1_LegControl>(jac_check_rr, torque_control_rr, print_error);
    auto leg_control_rl = make_ref<A1_LegControl>(jac_check_rl, torque_control_rl, print_error);

    auto mj = MuJoCoLaunch(MODEL_SCENE_PATH, a1->sim_data);
    *log << "Launching MuJoCo (Model: " << MODEL_SCENE_PATH << ")\n";
    if (!mj.start())
    {
        *log << "Failed to start MuJoCo.\n";
        return -1;
    }

    while (mj.ok())
    {
        mj.step(0.01);
        mj.render();

        // Update setpoint
        const scalar_t set_z = -0.27 + 0.05 * std::sin(a1->clock->time());
        translation3d_t toe_position_left = {0.0, A1_LS, set_z};
        translation3d_t toe_position_right = {0.0, -A1_LS, set_z};
        a1->control.front_right.toe_position_setpoint->write(toe_position_right);
        a1->control.front_left.toe_position_setpoint->write(toe_position_left);
        a1->control.rear_right.toe_position_setpoint->write(toe_position_right);
        a1->control.rear_left.toe_position_setpoint->write(toe_position_left);

        // Run OSC
        leg_control_fr->eval();
        leg_control_fl->eval();
        leg_control_rr->eval();
        leg_control_rl->eval();

        const scalar_t body_z = a1->state.body.position->read()[R3::Z];
        *log << "Z Setpoint: " << -set_z << ", Z body: " << body_z << ", Error: " << (-set_z - body_z) << "\n";
    } // sim loop

    *log << "Closed MuJoCo.\n";
    return 0;
}