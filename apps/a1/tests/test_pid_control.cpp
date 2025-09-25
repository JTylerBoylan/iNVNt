#include "a1/a1.hpp"
#include "iNVNt/control/pid.hpp"

using namespace nvn;
using namespace a1;
int main(int argc, char **argv)
{
    constexpr int N = A1_NUM_JOINTS;

    const std::string path = std::string(INVNT_SHARE_DIR) + "/a1/model/scene.xml";

    scalar_t freq = 100; // Hz
    auto Kp = vector_t<scalar_t, N>::Constant(100.0);
    auto Kd = vector_t<scalar_t, N>::Constant(10.0);
    auto Ki = vector_t<scalar_t, N>::Constant(1.0);

    auto &sim_data = mj::mjSimData();

    auto read_q = mj::ReadJointPositions<N>(&sim_data);
    auto compute_pid = ComputePIDControl<scalar_t, N, PID>(freq, Kp, Kd, Ki);

    auto set_tau = mj::SetJointTorques<N>(&sim_data);

    auto set_q = [&](const vector_t<radians_t, N> &q)
    { set_tau(compute_pid(q, read_q())); };

    const vector_t<radians_t, 12> target_positions = {0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8, 0, 0.9, -1.8};

    auto log = ConsoleLog();
    log << "Launching MuJoCo (Model: " << path << ")\n";
    if (mj::mjLoadModel(path) && mj::mjOpenWindow())
    {
        auto &sim_data = mj::mjSimData();
        while (!mj::mjShouldWindowClose())
        {
            mj::mjStep(0.01);
            mj::mjRender();

            set_q(target_positions);

            auto position = mj::ReadJointPosition(&sim_data, FR_HIP)();
            auto velocity = mj::ReadJointVelocity(&sim_data, FR_HIP)();
            log << "FR Hip Joint position: " << position << "\n";
            log << "FR Hip Joint velocity: " << velocity << "\n";
        } // sim loop
        log << "Closed MuJoCo.\n";
    }
    else
    {
        log << "Failed to start MuJoCo.\n";
        return -1;
    }

    return 0;
}