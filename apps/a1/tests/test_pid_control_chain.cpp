#include "a1/a1.hpp"
#include "iNVNt/control/pid.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    UnitreeA1 a1;
    std::string path = std::string(INVNT_SHARE_DIR) + "/a1/model/scene.xml";

    scalar_t freq = 100; // Hz
    auto Kp = vector_t<scalar_t, 12>::Constant(100.0);
    auto Kd = vector_t<scalar_t, 12>::Constant(10.0);
    auto Ki = vector_t<scalar_t, 12>::Constant(1.0);

    using A1_JointsPID = PIDControl<scalar_t, 12, PID, A1_ReadAllJointPositions>;
    auto joints_pid = A1_JointsPID(a1.state.joints.positions_vec, freq, Kp, Kd, Ki);
    static_assert(Callable<A1_JointsPID, vector_t<scalar_t, 12>>);

    static_assert(Chainable<A1_JointsPID, A1_SetAllJointTorques, vector_t<scalar_t, 12>>);
    using A1_SetAllJointPositions = Chain<A1_JointsPID, A1_SetAllJointTorques>;
    auto set_joints_pos = A1_SetAllJointPositions(joints_pid, a1.control.joints.torques_vec);
    static_assert(Callable<A1_SetAllJointPositions, vector_t<scalar_t, 12>>);

    vector_t<radians_t, 12> target_positions = {0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8};
    auto setpoints = ReadWrite<vector_t<radians_t, 12>>(target_positions);

    static_assert(Chainable<ReadWrite<vector_t<radians_t, 12>>, A1_SetAllJointPositions>);
    using A1_JointsPIDControl = Chain<ReadWrite<vector_t<radians_t, 12>>, A1_SetAllJointPositions>;
    auto pid_control = A1_JointsPIDControl(setpoints, set_joints_pos);
    static_assert(Callable<A1_JointsPIDControl>);

    a1.log << "Launching MuJoCo (Model: " << path << ")\n";
    if (mj::mjLoadModel(path) && mj::mjOpenWindow())
    {
        auto &sim_data = mj::mjSimData();
        while (!mj::mjShouldWindowClose())
        {
            mj::mjStep(0.01);
            mj::mjRender();

            pid_control();
            auto position = a1.state.joints.positions[FR_HIP]();
            a1.log << "Joint position: " << position << "\n";
            auto velocity = a1.state.joints.velocities[FR_HIP]();
            a1.log << "Joint velocity: " << velocity << "\n";
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