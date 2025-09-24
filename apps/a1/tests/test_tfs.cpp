#include "a1/a1.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    // Create A1 robot
    UnitreeA1 a1;

    // Launch A1 sim
    a1.log << "Launching MuJoCo...\n";
    a1.log << "  Model: " << MODEL_SCENE_PATH << "\n";

    if (!mj::mjLoadModel(MODEL_SCENE_PATH))
    {
        a1.log << "Error: Failed to load model!\n";
        return -1;
    }

    if (!mj::mjOpenWindow())
    {
        a1.log << "Error: Failed to open winoow!\n";
        return -1;
    }

    auto read_wTb = a1.tf("world", "base_link");
    auto read_bTfrt = a1.tf("base_link", "fr_toe");
    auto read_bTflt = a1.tf("base_link", "fl_toe");
    auto read_frtTflt = a1.tf("fr_toe", "fl_toe");

    auto &sim_data = mj::mjSimData();
    while (!mj::mjShouldWindowClose())
    {
        // Run physics & render
        mj::mjStep(0.01);
        mj::mjRender();

        auto w_T_b = read_wTb();
        auto b_T_frt = read_bTfrt();
        auto b_T_flt = read_bTflt();
        auto frt_T_flt = read_frtTflt();

        if (a1.clock.timer.seconds() > 5.0)
        {
            a1.log << "Body Position: " << w_T_b.translation().transpose() << "\n";
            a1.log << "Body Orientation: " << w_T_b.rotation().eulerAngles(2, 1, 0).transpose() << "\n";

            a1.log << "Front Right from Body: " << b_T_frt.translation().transpose() << "\n";
            a1.log << "Front Left from Body: " << b_T_flt.translation().transpose() << "\n";
            a1.log << "Front Left from Front Right: " << frt_T_flt.translation().transpose() << "\n";
            a1.clock.timer.reset();
        }

    } // sim loop
    a1.log << "Closed MuJoCo.\n";

    return 0;
}