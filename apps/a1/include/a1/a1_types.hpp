#pragma once

#include "iNVNt/core/core.hpp"
#include "iNVNt/sim/mujoco_functors.hpp"

namespace a1
{
    using namespace nvn;

    const static std::string SHARE_DIR = std::string(INVNT_SHARE_DIR) + "/a1";
    const static std::string MODEL_SCENE_PATH = SHARE_DIR + "/model/scene.xml";

    constexpr int A1_NUM_JOINTS = 12;
    constexpr int A1_NUM_LEGS = 4;
    constexpr int A1_NUM_JOINTS_PER_LEG = A1_NUM_JOINTS / A1_NUM_LEGS;
    constexpr bool A1_LEFT = false, A1_RIGHT = true;
    constexpr scalar_t A1_LS = 0.08505, A1_LT = 0.2, A1_LC = 0.2;
    constexpr meters_t A1_DEFAULT_HEIGHT = 0.27;
    constexpr meters_t A1_DEFAULT_YOFF = A1_LS;
    constexpr meters_t A1_DEFAULT_Y_OFFSET_RIGHT = -A1_LS;
    constexpr meters_t A1_DEFAULT_Y_OFFSET_LEFT = +A1_LS;

    enum A1_LegID : index_t
    {
        FR,
        FL,
        RR,
        RL
    };

    const auto A1_LEG_YDIR = [](A1_LegID leg_id) { return 2 * (leg_id & 0b1) - 1;};

    enum A1_JointID : index_t
    {
        FR_ABDUCT,
        FR_HIP,
        FR_KNEE,
        FL_ABDUCT,
        FL_HIP,
        FL_KNEE,
        RR_ABDUCT,
        RR_HIP,
        RR_KNEE,
        RL_ABDUCT,
        RL_HIP,
        RL_KNEE,
    };
}