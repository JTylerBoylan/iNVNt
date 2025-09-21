#include "a1/a1.hpp"
#include "a1/leg_control_chain.hpp"

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

    auto leg_control_fr = make_leg_control_chain(a1.state, a1.control, FR, freq, Kp, Kd, Ki);
    auto leg_control_fl = make_leg_control_chain(a1.state, a1.control, FL, freq, Kp, Kd, Ki);
    auto leg_control_rr = make_leg_control_chain(a1.state, a1.control, RR, freq, Kp, Kd, Ki);
    auto leg_control_rl = make_leg_control_chain(a1.state, a1.control, RL, freq, Kp, Kd, Ki);

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
            a1.control.legs[FR].position_setpoint(toe_position_right);
            a1.control.legs[FL].position_setpoint(toe_position_left);
            a1.control.legs[RR].position_setpoint(toe_position_right);
            a1.control.legs[RL].position_setpoint(toe_position_left);

            if (a1.clock() > 5.0 && a1.clock() < 5.015)
            {
                a1.log << "Switching Front Legs to Force Mode!\n";
                a1.control.legs[FR].is_in_stance(true);
                a1.control.legs[FL].is_in_stance(true);
                a1.control.legs[FR].force_setpoint({100.0, 0.0, -100.0});
                a1.control.legs[FL].force_setpoint({100.0, 0.0, -100.0});
            }

            // Run OSC
            leg_control_fr();
            leg_control_fl();
            leg_control_rr();
            leg_control_rl();

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