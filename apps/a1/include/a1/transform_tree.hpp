#pragma once

#include "a1/a1_state.hpp"
#include "a1/param_server.hpp"

namespace a1
{
    using namespace nvn;

    using A1_ReadJointQuaternion = ReadJointQuaternion<A1_ReadJointPosition &>;
    using A1_BodyTransform = Transform3D<A1_ReadBodyPosition &, A1_ReadBodyOrientation &>;
    using A1_TranslationTransform = Transform3D<TypedParam<translation3d_t> &, IdentityQuaternion>;
    using A1_OrientationTransform = Transform3D<IdentityTranslation3D, TypedParam<quaternion_t> &>;
    using A1_LinkJointTransform = Transform3D<TypedParam<translation3d_t> &, A1_ReadJointQuaternion &>;

    struct A1_TransformTree final : TransformTree
    {
        std::unique_ptr<TransformTreeGraph> graph;
        std::unordered_map<std::string, std::size_t> tf_name_map;

        A1_TransformTree(const A1_State &state,
                         const A1_ParameterServer &params)
            : graph(std::make_unique<TransformTreeGraph>()),
              TransformTree(graph.get())
        {
            // Read joint quaternion
            auto fr_abduct_quaternion = A1_ReadJointQuaternion(state.joints.positions[FR_ABDUCT]);
            auto fr_hip_quaternion = A1_ReadJointQuaternion(state.joints.positions[FR_HIP]);
            auto fr_knee_quaternion = A1_ReadJointQuaternion(state.joints.positions[FR_KNEE]);
            auto fl_abduct_quaternion = A1_ReadJointQuaternion(state.joints.positions[FL_ABDUCT]);
            auto fl_hip_quaternion = A1_ReadJointQuaternion(state.joints.positions[FL_HIP]);
            auto fl_knee_quaternion = A1_ReadJointQuaternion(state.joints.positions[FL_KNEE]);
            auto rr_abduct_quaternion = A1_ReadJointQuaternion(state.joints.positions[RR_ABDUCT]);
            auto rr_hip_quaternion = A1_ReadJointQuaternion(state.joints.positions[RR_HIP]);
            auto rr_knee_quaternion = A1_ReadJointQuaternion(state.joints.positions[RR_KNEE]);
            auto rl_abduct_quaternion = A1_ReadJointQuaternion(state.joints.positions[RL_ABDUCT]);
            auto rl_hip_quaternion = A1_ReadJointQuaternion(state.joints.positions[RL_HIP]);
            auto rl_knee_quaternion = A1_ReadJointQuaternion(state.joints.positions[RL_KNEE]);

            // Params
            auto fr_leg_offset = params.getParam<translation3d_t>("FR_leg_offset");
            auto fl_leg_offset = params.getParam<translation3d_t>("FL_leg_offset");
            auto rr_leg_offset = params.getParam<translation3d_t>("RR_leg_offset");
            auto rl_leg_offset = params.getParam<translation3d_t>("RL_leg_offset");
            auto abduction_orientation = params.getParam<quaternion_t>("abduction_orientation");
            auto hip_orientation = params.getParam<quaternion_t>("hip_orientation");
            auto knee_orientation = params.getParam<quaternion_t>("knee_orientation");
            auto toe_orientation = params.getParam<quaternion_t>("toe_orientation");
            auto shoulder_link_vector_right = params.getParam<translation3d_t>("shoulder_link_vector_right");
            auto shoulder_link_vector_left = params.getParam<translation3d_t>("shoulder_link_vector_left");
            auto thigh_link_vector = params.getParam<translation3d_t>("thigh_link_vector");
            auto calf_link_vector = params.getParam<translation3d_t>("calf_link_vector");

            // Transforms
            auto tf_world = IdentityTransform3D();
            auto tf_world_body = A1_BodyTransform(std::cref(state.odom.position), std::cref(state.odom.orientation));

            auto tf_body_fr = A1_TranslationTransform(std::cref(*fr_leg_offset), IdentityQuaternion());
            auto tf_body_fl = A1_TranslationTransform(std::cref(*fl_leg_offset), IdentityQuaternion());
            auto tf_body_rr = A1_TranslationTransform(std::cref(*rr_leg_offset), IdentityQuaternion());
            auto tf_body_rl = A1_TranslationTransform(std::cref(*rl_leg_offset), IdentityQuaternion());

            auto tf_ll_abduct = A1_OrientationTransform(IdentityTranslation3D(), std::cref(*abduction_orientation));
            auto tf_shoulder_hip = A1_OrientationTransform(IdentityTranslation3D(), std::cref(*hip_orientation));
            auto tf_thigh_knee = A1_OrientationTransform(IdentityTranslation3D(), std::cref(*knee_orientation));
            auto tf_calf_toe = A1_OrientationTransform(IdentityTranslation3D(), std::cref(*toe_orientation));

            auto tf_abduct_shoulder_fr = A1_LinkJointTransform(std::cref(*shoulder_link_vector_right), std::move(fr_abduct_quaternion));
            auto tf_hip_thigh_fr = A1_LinkJointTransform(std::cref(*thigh_link_vector), std::move(fr_hip_quaternion));
            auto tf_knee_calf_fr = A1_LinkJointTransform(std::cref(*calf_link_vector), std::move(fr_knee_quaternion));

            auto tf_abduct_shoulder_fl = A1_LinkJointTransform(std::cref(*shoulder_link_vector_left), std::move(fl_abduct_quaternion));
            auto tf_hip_thigh_fl = A1_LinkJointTransform(std::cref(*thigh_link_vector), std::move(fl_hip_quaternion));
            auto tf_knee_calf_fl = A1_LinkJointTransform(std::cref(*calf_link_vector), std::move(fl_knee_quaternion));

            auto tf_abduct_shoulder_rr = A1_LinkJointTransform(std::cref(*shoulder_link_vector_right), std::move(rr_abduct_quaternion));
            auto tf_hip_thigh_rr = A1_LinkJointTransform(std::cref(*thigh_link_vector), std::move(rr_hip_quaternion));
            auto tf_knee_calf_rr = A1_LinkJointTransform(std::cref(*calf_link_vector), std::move(rr_knee_quaternion));

            auto tf_abduct_shoulder_rl = A1_LinkJointTransform(std::cref(*shoulder_link_vector_left), std::move(rl_abduct_quaternion));
            auto tf_hip_thigh_rl = A1_LinkJointTransform(std::cref(*thigh_link_vector), std::move(rl_hip_quaternion));
            auto tf_knee_calf_rl = A1_LinkJointTransform(std::cref(*calf_link_vector), std::move(rl_knee_quaternion));

            // Transform frames
            tf_name_map["world"] = graph->addNode(tf_world);
            tf_name_map["base_link"] = graph->addNode(tf_world_body);
            tf_name_map["fr_link"] = graph->addNode(tf_body_fr, tf_name_map["base_link"]);
            tf_name_map["fr_abduct"] = graph->addNode(tf_ll_abduct, tf_name_map["fr_link"]);
            tf_name_map["fr_shoulder"] = graph->addNode(tf_abduct_shoulder_fr, tf_name_map["fr_abduct"]);
            tf_name_map["fr_hip"] = graph->addNode(tf_shoulder_hip, tf_name_map["fr_shoulder"]);
            tf_name_map["fr_thigh"] = graph->addNode(tf_hip_thigh_fr, tf_name_map["fr_hip"]);
            tf_name_map["fr_knee"] = graph->addNode(tf_thigh_knee, tf_name_map["fr_thigh"]);
            tf_name_map["fr_calf"] = graph->addNode(tf_knee_calf_fr, tf_name_map["fr_knee"]);
            tf_name_map["fr_toe"] = graph->addNode(tf_calf_toe, tf_name_map["fr_calf"]);
            tf_name_map["fl_link"] = graph->addNode(tf_body_fl, tf_name_map["base_link"]);
            tf_name_map["fl_abduct"] = graph->addNode(tf_ll_abduct, tf_name_map["fl_link"]);
            tf_name_map["fl_shoulder"] = graph->addNode(tf_abduct_shoulder_fl, tf_name_map["fl_abduct"]);
            tf_name_map["fl_hip"] = graph->addNode(tf_shoulder_hip, tf_name_map["fl_shoulder"]);
            tf_name_map["fl_thigh"] = graph->addNode(tf_hip_thigh_fl, tf_name_map["fl_hip"]);
            tf_name_map["fl_knee"] = graph->addNode(tf_thigh_knee, tf_name_map["fl_thigh"]);
            tf_name_map["fl_calf"] = graph->addNode(tf_knee_calf_fl, tf_name_map["fl_knee"]);
            tf_name_map["fl_toe"] = graph->addNode(tf_calf_toe, tf_name_map["fl_calf"]);
            tf_name_map["rr_link"] = graph->addNode(tf_body_rr, tf_name_map["base_link"]);
            tf_name_map["rr_abduct"] = graph->addNode(tf_ll_abduct, tf_name_map["rr_link"]);
            tf_name_map["rr_shoulder"] = graph->addNode(tf_abduct_shoulder_rr, tf_name_map["rr_abduct"]);
            tf_name_map["rr_hip"] = graph->addNode(tf_shoulder_hip, tf_name_map["rr_shoulder"]);
            tf_name_map["rr_thigh"] = graph->addNode(tf_hip_thigh_rr, tf_name_map["rr_hip"]);
            tf_name_map["rr_knee"] = graph->addNode(tf_thigh_knee, tf_name_map["rr_thigh"]);
            tf_name_map["rr_calf"] = graph->addNode(tf_knee_calf_rr, tf_name_map["rr_knee"]);
            tf_name_map["rr_toe"] = graph->addNode(tf_calf_toe, tf_name_map["rr_calf"]);
            tf_name_map["rl_link"] = graph->addNode(tf_body_rl, tf_name_map["base_link"]);
            tf_name_map["rl_abduct"] = graph->addNode(tf_ll_abduct, tf_name_map["rl_link"]);
            tf_name_map["rl_shoulder"] = graph->addNode(tf_abduct_shoulder_rl, tf_name_map["rl_abduct"]);
            tf_name_map["rl_hip"] = graph->addNode(tf_shoulder_hip, tf_name_map["rl_shoulder"]);
            tf_name_map["rl_thigh"] = graph->addNode(tf_hip_thigh_rl, tf_name_map["rl_hip"]);
            tf_name_map["rl_knee"] = graph->addNode(tf_thigh_knee, tf_name_map["rl_thigh"]);
            tf_name_map["rl_calf"] = graph->addNode(tf_knee_calf_rl, tf_name_map["rl_knee"]);
            tf_name_map["rl_toe"] = graph->addNode(tf_calf_toe, tf_name_map["rl_calf"]);
        }
    };
}