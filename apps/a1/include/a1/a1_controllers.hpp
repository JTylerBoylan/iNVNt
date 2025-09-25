#pragma once

#include "a1/a1_state.hpp"
#include "a1/a1_control.hpp"
#include "a1/param_server.hpp"
#include "a1/a1_toe_control.hpp"

namespace a1
{
    struct A1_Controllers
    {
        std::array<A1_ToeController, A1_NUM_LEGS> toe_ctrl;

        A1_Controllers(const A1_State &state, const A1_Control &control,
                       const A1_ParameterServer &params)
            : toe_ctrl({A1_ToeController(state.legs[FR], control.legs[FR], params),
                        A1_ToeController(state.legs[FL], control.legs[FL], params),
                        A1_ToeController(state.legs[RR], control.legs[RR], params),
                        A1_ToeController(state.legs[RL], control.legs[RL], params)}) {}
    };
}