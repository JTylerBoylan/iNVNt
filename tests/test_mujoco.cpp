#include "iNVNt/core/logger.hpp"
#include "iNVNt/sim/mujoco_functors.hpp"
#include "iNVNt/sim/mujoco_interface.hpp"

#include "iNVNt/core/config.hpp"

using namespace nvn;

void printSimData(SimData *sim_data);

auto s_log = Logger();
int main(int argc, char **argv)
{
    std::string path = std::string(INVNT_SHARE_DIR) + "/a1/model/scene.xml";
    
    s_log << "Launching MuJoCo (Model: " << path << ")\n";
    if (mj::mjLoadModel(path) && mj::mjOpenWindow())
    {
        auto &sim_data = mj::mjSimData();
        while (!mj::mjShouldWindowClose())
        {
            mj::mjStep(0.01);
            mj::mjRender();
            printSimData(&sim_data);
        } // sim loop
        s_log << "Closed MuJoCo.\n";
    }
    else
    {
        s_log << "Failed to start MuJoCo.\n";
        return -1;
    }

    return 0;
};

void printSimData(SimData *sim_data)
{
    auto position = sim_data->position;
    s_log << "Body position: " << position[0] << " " << position[1] << " " << position[2] << "\n";
    auto joint_pos = sim_data->joint_positions[0];
    s_log << "Joint position: " << joint_pos << "\n";
    auto joint_vel = sim_data->joint_velocities[0];
    s_log << "Joint velocity: " << joint_vel << "\n";
    sim_data->joint_torques[0] = 10.0; // Example torque
    s_log << "Set joint torque to 10.0 Nm\n";
}