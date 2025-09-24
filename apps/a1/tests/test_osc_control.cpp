#include "a1/a1.hpp"
#include "iNVNt/control/kinematics.hpp"
#include "iNVNt/control/pid.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    UnitreeA1 a1;
    std::string path = std::string(INVNT_SHARE_DIR) + "/a1/model/scene.xml";

    scalar_t freq = 100; // Hz
    auto Kp = vector_t<scalar_t, 3>::Constant(1000.0);
    auto Kd = vector_t<scalar_t, 3>::Constant(10.0);
    auto Ki = vector_t<scalar_t, 3>::Constant(1.0);

    using A1_JacobianMap = JacobianMap<A1_ReadToeJacobian3D &>;
    auto jac_map_fr = A1_JacobianMap(a1.state.legs[FR].toe_jacobian);
    auto jac_map_fl = A1_JacobianMap(a1.state.legs[FL].toe_jacobian);
    auto jac_map_rr = A1_JacobianMap(a1.state.legs[RR].toe_jacobian);
    auto jac_map_rl = A1_JacobianMap(a1.state.legs[RL].toe_jacobian);

    auto set_force_fr = Chain(std::ref(jac_map_fr), std::ref(a1.control.legs[FR].joint_torques));
    auto set_force_fl = Chain(std::ref(jac_map_fl), std::ref(a1.control.legs[FL].joint_torques));
    auto set_force_rr = Chain(std::ref(jac_map_rr), std::ref(a1.control.legs[RR].joint_torques));
    auto set_force_rl = Chain(std::ref(jac_map_rl), std::ref(a1.control.legs[RL].joint_torques));

    using A1_LegPID_Compute = PID<scalar_t, 3, A1_ReadToePosition>;
    auto leg_pid_fr = A1_LegPID_Compute(a1.state.legs[FR].toe_position, freq, Kp, Kd, Ki);
    auto leg_pid_fl = A1_LegPID_Compute(a1.state.legs[FL].toe_position, freq, Kp, Kd, Ki);
    auto leg_pid_rr = A1_LegPID_Compute(a1.state.legs[RR].toe_position, freq, Kp, Kd, Ki);
    auto leg_pid_rl = A1_LegPID_Compute(a1.state.legs[RL].toe_position, freq, Kp, Kd, Ki);

    auto set_leg_pos_fr = Chain(leg_pid_fr, set_force_fr);
    auto set_leg_pos_fl = Chain(leg_pid_fl, set_force_fl);
    auto set_leg_pos_rr = Chain(leg_pid_rr, set_force_rr);
    auto set_leg_pos_rl = Chain(leg_pid_rl, set_force_rl);

    auto osc_control_fr = Chain(std::ref(a1.controllers.toe_ctrl[FR].pos_setpoint), set_leg_pos_fr);
    auto osc_control_fl = Chain(std::ref(a1.controllers.toe_ctrl[FL].pos_setpoint), set_leg_pos_fl);
    auto osc_control_rr = Chain(std::ref(a1.controllers.toe_ctrl[RR].pos_setpoint), set_leg_pos_rr);
    auto osc_control_rl = Chain(std::ref(a1.controllers.toe_ctrl[RL].pos_setpoint), set_leg_pos_rl);

    a1.log << "Launching MuJoCo (Model: " << path << ")\n";
    if (mj::mjLoadModel(path) && mj::mjOpenWindow())
    {
        auto &sim_data = mj::mjSimData();
        while (!mj::mjShouldWindowClose())
        {
            mj::mjStep(0.01);
            mj::mjRender();

            // Update setpoint
            const scalar_t z_pos = -0.27 + 0.05 * std::sin(a1.clock());

            translation3d_t toe_position_left = {0.0, A1_LS, z_pos};
            translation3d_t toe_position_right = {0.0, -A1_LS, z_pos};
            a1.controllers.toe_ctrl[FR].pos_setpoint(toe_position_right);
            a1.controllers.toe_ctrl[FL].pos_setpoint(toe_position_left);
            a1.controllers.toe_ctrl[RR].pos_setpoint(toe_position_right);
            a1.controllers.toe_ctrl[RL].pos_setpoint(toe_position_left);

            // Run OSC
            osc_control_fr();
            osc_control_fl();
            osc_control_rr();
            osc_control_rl();

            const scalar_t body_z = a1.state.odom.position().z();
            a1.log << "Z Setpoint: " << -z_pos << ", Z body: " << body_z << ", Error: " << (-z_pos - body_z) << "\n";
        } // sim loop
        a1.log << "Closed MuJoCo.\n";
    }
    else
    {
        a1.log << "Failed to start MuJoCo.\n";
        return -1;
    }

    return 0;
}