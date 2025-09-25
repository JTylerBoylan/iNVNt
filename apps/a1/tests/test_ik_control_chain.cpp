#include "a1/a1.hpp"
#include "iNVNt/control/pid.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    UnitreeA1 a1;
    std::string path = std::string(INVNT_SHARE_DIR) + "/a1/model/scene.xml";

    scalar_t freq = 100; // Hz
    auto Kp = vector_t<scalar_t, 3>::Constant(100.0);
    auto Kd = vector_t<scalar_t, 3>::Constant(10.0);
    auto Ki = vector_t<scalar_t, 3>::Constant(1.0);

    std::array<A1_LegPositionSetpoint, A1_NUM_LEGS> toe_setpoints({
        A1_LegPositionSetpoint({0.0, -A1_DEFAULT_YOFF, -A1_DEFAULT_HEIGHT}),
        A1_LegPositionSetpoint({0.0, +A1_DEFAULT_YOFF, -A1_DEFAULT_HEIGHT}),
        A1_LegPositionSetpoint({0.0, -A1_DEFAULT_YOFF, -A1_DEFAULT_HEIGHT}),
        A1_LegPositionSetpoint({0.0, +A1_DEFAULT_YOFF, -A1_DEFAULT_HEIGHT})
    });

    using A1_LegJointsPID = PIDControl<scalar_t, 3, PID, A1_ReadLegJointPositions>;
    auto leg_joints_pid_fr = A1_LegJointsPID(a1.state.legs[FR].joint_positions, freq, Kp, Kd, Ki);
    auto leg_joints_pid_fl = A1_LegJointsPID(a1.state.legs[FL].joint_positions, freq, Kp, Kd, Ki);
    auto leg_joints_pid_rr = A1_LegJointsPID(a1.state.legs[RR].joint_positions, freq, Kp, Kd, Ki);
    auto leg_joints_pid_rl = A1_LegJointsPID(a1.state.legs[RL].joint_positions, freq, Kp, Kd, Ki);

    auto set_leg_joints_pos_fr = Chain(leg_joints_pid_fr, a1.control.legs[FR].joint_torques);
    auto set_leg_joints_pos_fl = Chain(leg_joints_pid_fl, a1.control.legs[FL].joint_torques);
    auto set_leg_joints_pos_rr = Chain(leg_joints_pid_rr, a1.control.legs[RR].joint_torques);
    auto set_leg_joints_pos_rl = Chain(leg_joints_pid_rl, a1.control.legs[RL].joint_torques);

    auto ik_compute_left = ComputeInverseKinematics(A1_LEFT, A1_LS, A1_LT, A1_LC);
    auto ik_compute_right = ComputeInverseKinematics(A1_RIGHT, A1_LS, A1_LT, A1_LC);

    auto set_leg_pos_fr = Chain(ik_compute_right, set_leg_joints_pos_fr);
    auto set_leg_pos_fl = Chain(ik_compute_left, set_leg_joints_pos_fl);
    auto set_leg_pos_rr = Chain(ik_compute_right, set_leg_joints_pos_rr);
    auto set_leg_pos_rl = Chain(ik_compute_left, set_leg_joints_pos_rl);

    auto ik_control_fr = Chain(std::ref(toe_setpoints[FR]), set_leg_pos_fr);
    auto ik_control_fl = Chain(std::ref(toe_setpoints[FL]), set_leg_pos_fl);
    auto ik_control_rr = Chain(std::ref(toe_setpoints[RR]), set_leg_pos_rr);
    auto ik_control_rl = Chain(std::ref(toe_setpoints[RL]), set_leg_pos_rl);

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
            a1.log << "Z position: " << z_pos << "\n";

            translation3d_t toe_position_left = {0.0, A1_LS, z_pos};
            translation3d_t toe_position_right = {0.0, -A1_LS, z_pos};
            toe_setpoints[FR](toe_position_right);
            toe_setpoints[FL](toe_position_left);
            toe_setpoints[RR](toe_position_right);
            toe_setpoints[RL](toe_position_left);

            // IK Control
            ik_control_fr();
            ik_control_fl();
            ik_control_rr();
            ik_control_rl();

            auto position = a1.state.legs[FR].toe_position();
            a1.log << position.transpose() << "\n";
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