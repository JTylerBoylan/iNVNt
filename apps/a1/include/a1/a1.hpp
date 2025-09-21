#pragma once

#include "a1/a1_types.hpp"
#include "a1/a1_state.hpp"
#include "a1/param_server.hpp"
#include "a1/transform_tree.hpp"

namespace a1
{
    using A1_Log = ConsoleLog;
    using A1_Clock = Clock<mj::ReadSimTime>;

    struct UnitreeA1
    {
        SimData *sim_data;

        A1_Log log;
        A1_Clock clock;
        A1_State state;
        A1_Control control;
        A1_ParameterServer params;
        A1_TransformTree tf;

        UnitreeA1()
            : sim_data(&mj::mjSimData()),
              log(),
              clock(sim_data),
              state(sim_data),
              control(sim_data),
              params(),
              tf(state, params) {}
    };
}