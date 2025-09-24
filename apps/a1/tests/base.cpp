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

    auto &sim_data = mj::mjSimData();
    while (!mj::mjShouldWindowClose())
    {
        // Run physics & render
        mj::mjStep(0.01);
        mj::mjRender();

        /* Control */

    } // sim loop
    a1.log << "Closed MuJoCo.\n";

    return 0;
}