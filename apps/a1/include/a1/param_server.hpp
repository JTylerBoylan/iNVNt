#pragma once

#include "iNVNt/core/param_server.hpp"
#include "iNVNt/core/math_types.hpp"

namespace a1
{
    using namespace nvn;

    struct A1_ParameterServer final : ParameterServer
    {
        A1_ParameterServer()
        {
            // setParamValue("mass", kilogram_t{10.75});
            // setParamValue("inertia", inertia_t{0.25, 0, 0, 0, 1.0, 0, 0, 0, 0.5});
            // setParamValue("gravity", vector_t<scalar_t, 3>{0.0, 0.0, -9.81});
            // setParamValue("foot_friction", scalar_t{0.8});
            setParamValue("default_stand_height", meters_t{0.27});
            // setParamValue("z_force_min", newtons_t{10.0});
            // setParamValue("z_force_max", newtons_t{250.0});
            setParamValue("leg_offset_x", meters_t{0.183});
            setParamValue("leg_offset_y", meters_t{0.047});
            setParamValue("FR_leg_offset", translation3d_t{0.183, -0.047, 0.0});
            setParamValue("FL_leg_offset", translation3d_t{0.183, 0.047, 0.0});
            setParamValue("RR_leg_offset", translation3d_t{-0.183, -0.047, 0.0});
            setParamValue("RL_leg_offset", translation3d_t{-0.183, 0.047, 0.0});
            setParamValue("abduction_orientation", quaternion_t{+0.5, -0.5, +0.5, -0.5});
            setParamValue("hip_orientation", quaternion_t{+0.5, -0.5, -0.5, +0.5});
            setParamValue("knee_orientation", quaternion_t{+1.0, +0.0, +0.0, +0.0});
            setParamValue("toe_orientation", quaternion_t{+0.5, +0.5, -0.5, -0.5});
            setParamValue("shoulder_length", meters_t{0.08505});
            setParamValue("thigh_length", meters_t{0.2});
            setParamValue("calf_length", meters_t{0.2});
            setParamValue("shoulder_link_vector_right", translation3d_t{0.08505, 0.0, 0.0});
            setParamValue("shoulder_link_vector_left", translation3d_t{-0.08505, 0.0, 0.0});
            setParamValue("thigh_link_vector", translation3d_t{0.2, 0.0, 0.0});
            setParamValue("calf_link_vector", translation3d_t{0.2, 0.0, 0.0});
        }
    };
}