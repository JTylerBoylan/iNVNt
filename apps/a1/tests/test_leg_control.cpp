#include "a1/a1.hpp"
#include "iNVNt/control/pid.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    UnitreeA1 a1;

    a1.log << "Launching MuJoCo (Model: " << a1::MODEL_SCENE_PATH << ")\n";
    if (mj::mjLoadModel(a1::MODEL_SCENE_PATH) && mj::mjOpenWindow())
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

            if (a1.clock() > 5.0 && a1.clock() < 5.05)
            {
                a1.log << "Switching Front Legs to Force Mode!\n";
                a1.controllers.toe_ctrl[FR].is_in_stance(true);
                a1.controllers.toe_ctrl[FL].is_in_stance(true);
                a1.controllers.toe_ctrl[FR].force_setpoint({100.0, 0.0, -100.0});
                a1.controllers.toe_ctrl[FL].force_setpoint({100.0, 0.0, -100.0});
            }

            // Run toe controller
            a1.controllers.toe_ctrl[FR]();
            a1.controllers.toe_ctrl[FL]();
            a1.controllers.toe_ctrl[RR]();
            a1.controllers.toe_ctrl[RL]();

            const scalar_t body_z = a1.state.odom.position().z();
            // a1.log << "Z Setpoint: " << -z_pos << ", Z body: " << body_z << ", Error: " << (-z_pos - body_z) << "\n";
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