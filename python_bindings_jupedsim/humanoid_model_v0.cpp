// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "conversion.hpp"
#include "wrapper.hpp"

#include <jupedsim/jupedsim.h>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_humanoid_model_v0(py::module_& m)
{
    py::class_<JPS_HumanoidModelV0AgentParameters>(m, "HumanoidModelV0AgentParameters")
        .def(
            py::init([](std::tuple<double, double> position,
                        std::tuple<double, double> orientation,
                        JPS_JourneyId journey_id,
                        JPS_StageId stage_id,
                        std::tuple<double, double> velocity,
                        double mass,
                        double desiredSpeed,
                        double reactionTime,
                        double agentScale,
                        double obstacleScale,
                        double forceDistance,
                        double radius,
                        // humanoid body parameters
                        double height,
                        // gait variables
                        int step_duration,
                        int step_timer,
                        int stepping_foot_index,
                        std::tuple<double, double> Xcom,
                        // humanoid body variables
                        std::tuple<double, double, double> head_position, 
                        std::tuple<double, double, double> pelvis_position,
                        double pelvis_rotation_angle_z,
                        double shoulder_rotation_angle_z,
                        double trunk_rotation_angle_x,
                        double trunk_rotation_angle_y,
                        std::tuple<double, double, double> heel_right_position, 
                        std::tuple<double, double, double> heel_left_position,
                        std::tuple<double, double, double> toe_right_position,
                        std::tuple<double, double, double> toe_left_position
                        ) 
                        {
                return JPS_HumanoidModelV0AgentParameters{
                    intoJPS_Point(position),
                    intoJPS_Point(orientation),
                    journey_id,
                    stage_id,
                    intoJPS_Point(velocity),
                    mass,
                    desiredSpeed,
                    reactionTime,
                    agentScale,
                    obstacleScale,
                    forceDistance,
                    radius, 
                    height,
                    step_duration,
                    step_timer,
                    stepping_foot_index,
                    intoJPS_Point(Xcom),
                    intoJPS_Point3D(head_position),
                    intoJPS_Point3D(pelvis_position),
                    pelvis_rotation_angle_z,
                    shoulder_rotation_angle_z,
                    trunk_rotation_angle_x,
                    trunk_rotation_angle_y,
                    intoJPS_Point3D(heel_right_position),
                    intoJPS_Point3D(heel_left_position),
                    intoJPS_Point3D(toe_right_position),
                    intoJPS_Point3D(toe_left_position)
                    };
            }),
            py::kw_only(),
            py::arg("position"),
            py::arg("orientation"),
            py::arg("journey_id"),
            py::arg("stage_id"),
            py::arg("velocity"),
            py::arg("mass"),
            py::arg("desiredSpeed"),
            py::arg("reactionTime"),
            py::arg("agentScale"),
            py::arg("obstacleScale"),
            py::arg("forceDistance"),
            py::arg("radius"),
            py::arg("height"),
            py::arg("step_duration"),
            py::arg("step_timer"),
            py::arg("stepping_foot_index"),
            py::arg("Xcom"),
            py::arg("head_position"),
            py::arg("pelvis_position"),
            py::arg("pelvis_rotation_angle_z"),
            py::arg("shoulder_rotation_angle_z"),
            py::arg("trunk_rotation_angle_x"),
            py::arg("trunk_rotation_angle_y"),
            py::arg("heel_right_position"),
            py::arg("heel_left_position"),
            py::arg("toe_right_position"),
            py::arg("toe_left_position")
            )
        .def("__repr__", [](const JPS_HumanoidModelV0AgentParameters& p) {
            return fmt::format(
                "position: {}, orientation: {}, journey_id: {}, stage_id: {},"
                "velocity: {}, mass: {}, desiredSpeed: {},"
                "reactionTime: {}, agentScale: {}, obstacleScale: {}, forceDistance: {},"
                "radius: {}, height: {}, step_duration: {}, step_timer: {}, stepping_foot_index: {}, Xcom: {},"
                "head_position: {}, pelvis_position: {}, pelvis_rotation_angle_z: {},"
                "shoulder_rotation_angle_z: {}, trunk_rotation_angle_x: {},"
                "trunk_rotation_angle_y: {}, heel_right_position: {}, heel_left_position: {},"
                "toe_right_position: {}, toe_left_position: {}",
                intoTuple(p.position),
                intoTuple(p.orientation),
                p.journeyId,
                p.stageId,
                intoTuple(p.velocity),
                p.mass,
                p.desiredSpeed,
                p.reactionTime,
                p.agentScale,
                p.obstacleScale,
                p.forceDistance,
                p.radius,
                p.height,
                p.step_duration,
                p.step_timer,
                p.stepping_foot_index,
                intoTuple(p.Xcom),
                intoTuple3D(p.head_position),
                intoTuple3D(p.pelvis_position),
                p.pelvis_rotation_angle_z,
                p.shoulder_rotation_angle_z,
                p.trunk_rotation_angle_x,
                p.trunk_rotation_angle_y,
                intoTuple3D(p.heel_right_position),
                intoTuple3D(p.heel_left_position),
                intoTuple3D(p.toe_right_position),
                intoTuple3D(p.toe_left_position)
                );
        });
    py::class_<JPS_HumanoidModelV0Builder_Wrapper>(m, "HumanoidModelV0Builder")
        .def(
            py::init([](double bodyForce, double friction) {
                return std::make_unique<JPS_HumanoidModelV0Builder_Wrapper>(
                    JPS_HumanoidModelV0Builder_Create(bodyForce, friction));
            }),
            py::kw_only(),
            py::arg("bodyForce"),
            py::arg("friction"))
        .def("build", [](JPS_HumanoidModelV0Builder_Wrapper& w) {
            JPS_ErrorMessage errorMsg{};
            auto result = JPS_HumanoidModelV0Builder_Build(w.handle, &errorMsg);
            if(result) {
                return std::make_unique<JPS_OperationalModel_Wrapper>(result);
            }
            auto msg = std::string(JPS_ErrorMessage_GetMessage(errorMsg));
            JPS_ErrorMessage_Free(errorMsg);
            throw std::runtime_error{msg};
        });
    py::class_<JPS_HumanoidModelV0State_Wrapper>(m, "HumanoidModelV0State")
        .def_property(
            "velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> velocity) {
                JPS_HumanoidModelV0State_SetVelocity(w.handle, intoJPS_Point(velocity));
            })
        .def_property(
            "mass",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetMass(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double mass) {
                JPS_HumanoidModelV0State_SetMass(w.handle, mass);
            })
        .def_property(
            "desiredSpeed",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetDesiredSpeed(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double desiredSpeed) {
                JPS_HumanoidModelV0State_SetDesiredSpeed(w.handle, desiredSpeed);
            })
        .def_property(
            "reactionTime",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetReactionTime(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double reactionTime) {
                JPS_HumanoidModelV0State_SetReactionTime(w.handle, reactionTime);
            })
        .def_property(
            "agentScale",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetAgentScale(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double agentScale) {
                JPS_HumanoidModelV0State_SetAgentScale(w.handle, agentScale);
            })
        .def_property(
            "obstacleScale",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetObstacleScale(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double obstacleScale) {
                JPS_HumanoidModelV0State_SetObstacleScale(w.handle, obstacleScale);
            })
        .def_property(
            "ForceDistance",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetForceDistance(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double forceDistance) {
                JPS_HumanoidModelV0State_SetForceDistance(w.handle, forceDistance);
            })
        .def_property(
            "radius",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetRadius(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double radius) {
                JPS_HumanoidModelV0State_SetRadius(w.handle, radius);
            })
        .def_property(
            "height",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetHeight(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double height) {
                JPS_HumanoidModelV0State_SetHeight(w.handle, height);
            })
        .def_property(
            "step_duration",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetStepDuration(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, int step_duration) {
                JPS_HumanoidModelV0State_SetStepDuration(w.handle, step_duration);
            })
        .def_property(
            "step_timer",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetStepTimer(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, int step_timer) {
                JPS_HumanoidModelV0State_SetStepTimer(w.handle, step_timer);
            })
        .def_property(
            "stepping_foot_index",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetSteppingFootIndex(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, int stepping_foot_index) {
                JPS_HumanoidModelV0State_SetSteppingFootIndex(w.handle, stepping_foot_index);
            })
        .def_property(
            "Xcom",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetXcom(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> Xcom) {
                JPS_HumanoidModelV0State_SetXcom(w.handle, intoJPS_Point(Xcom));
            })
        .def_property(
            "head_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetHeadPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> head_position) {
                JPS_HumanoidModelV0State_SetHeadPosition(w.handle, intoJPS_Point3D(head_position));
            })
        .def_property(
            "pelvis_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetPelvisPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> pelvis_position) {
                JPS_HumanoidModelV0State_SetPelvisPosition(w.handle, intoJPS_Point3D(pelvis_position));
            })
        .def_property(
            "pelvis_rotation_angle_z",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetPelvisRotationAngleZ(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double pelvis_rotation_angle_z) {
                JPS_HumanoidModelV0State_SetPelvisRotationAngleZ(w.handle, pelvis_rotation_angle_z);
            })
        .def_property(
            "shoulder_rotation_angle_z",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetShoulderRotationAngleZ(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double shoulder_rotation_angle_z) {
                JPS_HumanoidModelV0State_SetShoulderRotationAngleZ(w.handle, shoulder_rotation_angle_z);
            })
        .def_property(
            "trunk_rotation_angle_x",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetTrunkRotationAngleX(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double trunk_rotation_angle_x) {
                JPS_HumanoidModelV0State_SetTrunkRotationAngleX(w.handle, trunk_rotation_angle_x);
            })
        .def_property(
            "trunk_rotation_angle_y",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetTrunkRotationAngleY(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double trunk_rotation_angle_y) {
                JPS_HumanoidModelV0State_SetTrunkRotationAngleY(w.handle, trunk_rotation_angle_y);
            })
        .def_property(
            "heel_right_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetHeelRightPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> heel_right_position) {
                JPS_HumanoidModelV0State_SetHeelRightPosition(w.handle, intoJPS_Point3D(heel_right_position));
            })
        .def_property(
            "heel_left_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetHeelLeftPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> heel_left_position) {
                JPS_HumanoidModelV0State_SetHeelLeftPosition(w.handle, intoJPS_Point3D(heel_left_position));
            })
        .def_property(
            "toe_right_position",   
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetToeRightPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> toe_right_position) {
                JPS_HumanoidModelV0State_SetToeRightPosition(w.handle, intoJPS_Point3D(toe_right_position));
            })
        .def_property(
            "toe_left_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple3D(JPS_HumanoidModelV0State_GetToeLeftPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double, double> toe_left_position) {
                JPS_HumanoidModelV0State_SetToeLeftPosition(w.handle, intoJPS_Point3D(toe_left_position));
            })
            ;
}
