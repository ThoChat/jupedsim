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
                        // humanoid body variables
                        std::tuple<double, double> head_position, 
                        std::tuple<double, double> head_velocity,
                        double shoulder_rotation_angle_z,
                        double shoulder_rotation_velocity_z,
                        double trunk_rotation_angle_x,
                        double trunk_rotation_velocity_x,
                        double trunk_rotation_angle_y,
                        double trunk_rotation_velocity_y,
                        std::tuple<double, double> heel_right_position, 
                        std::tuple<double, double> heel_right_velocity,
                        std::tuple<double, double> heel_left_position, 
                        std::tuple<double, double> heel_left_velocity
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
                    intoJPS_Point(head_position),
                    intoJPS_Point(head_velocity),
                    shoulder_rotation_angle_z,
                    shoulder_rotation_velocity_z,
                    trunk_rotation_angle_x,
                    trunk_rotation_velocity_x,
                    trunk_rotation_angle_y,
                    trunk_rotation_velocity_y,
                    intoJPS_Point(heel_right_position),
                    intoJPS_Point(heel_right_velocity),
                    intoJPS_Point(heel_left_position),
                    intoJPS_Point(heel_left_velocity)
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
            py::arg("head_position"),
            py::arg("head_velocity"),
            py::arg("shoulder_rotation_angle_z"),
            py::arg("shoulder_rotation_velocity_z"),
            py::arg("trunk_rotation_angle_x"),
            py::arg("trunk_rotation_velocity_x"),
            py::arg("trunk_rotation_angle_y"),
            py::arg("trunk_rotation_velocity_y"),
            py::arg("heel_right_position"),
            py::arg("heel_right_velocity"),
            py::arg("heel_left_position"),
            py::arg("heel_left_velocity")
            )
        .def("__repr__", [](const JPS_HumanoidModelV0AgentParameters& p) {
            return fmt::format(
                "position: {}, orientation: {}, journey_id: {}, stage_id: {},"
                "velocity: {}, mass: {}, desiredSpeed: {},"
                "reactionTime: {}, agentScale: {}, obstacleScale: {}, forceDistance: {},"
                "radius: {}, height: {}, head_position: {}, head_velocity: {}, shoulder_rotation_angle_z: {},"
                "shoulder_rotation_velocity_z: {}, trunk_rotation_angle_x: {}, trunk_rotation_velocity_x: {},"
                "trunk_rotation_angle_y: {}, trunk_rotation_velocity_y: {},"
                "heel_right_position: {}, heel_right_velocity: {}, heel_left_position: {}, heel_left_velocity: {},",
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
                intoTuple(p.head_position),
                intoTuple(p.head_velocity),
                p.shoulder_rotation_angle_z,
                p.shoulder_rotation_velocity_z,
                p.trunk_rotation_angle_x,
                p.trunk_rotation_velocity_x,
                p.trunk_rotation_angle_y,
                p.trunk_rotation_velocity_y,
                intoTuple(p.heel_right_position),
                intoTuple(p.heel_right_velocity),
                intoTuple(p.heel_left_position),
                intoTuple(p.heel_left_velocity)
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
            "head_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeadPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> head_position) {
                JPS_HumanoidModelV0State_SetHeadPosition(w.handle, intoJPS_Point(head_position));
            })
        .def_property(
            "head_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeadVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> head_velocity) {
                JPS_HumanoidModelV0State_SetHeadVelocity(w.handle, intoJPS_Point(head_velocity));
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
            "shoulder_rotation_velocity_z",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetShoulderRotationVelocityZ(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double shoulder_rotation_velocity_z) {
                JPS_HumanoidModelV0State_SetShoulderRotationVelocityZ(w.handle, shoulder_rotation_velocity_z);
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
            "trunk_rotation_velocity_x",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetTrunkRotationVelocityX(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double trunk_rotation_velocity_x) {
                JPS_HumanoidModelV0State_SetTrunkRotationVelocityX(w.handle, trunk_rotation_velocity_x);
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
            "trunk_rotation_velocity_y",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return JPS_HumanoidModelV0State_GetTrunkRotationVelocityY(w.handle);
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, double trunk_rotation_velocity_y) {
                JPS_HumanoidModelV0State_SetTrunkRotationVelocityY(w.handle, trunk_rotation_velocity_y);
            })
        .def_property(
            "heel_right_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeelRightPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> heel_right_position) {
                JPS_HumanoidModelV0State_SetHeelRightPosition(w.handle, intoJPS_Point(heel_right_position));
            })
        .def_property(
            "heel_right_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeelRightVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> heel_right_velocity) {
                JPS_HumanoidModelV0State_SetHeelRightVelocity(w.handle, intoJPS_Point(heel_right_velocity));
            })
        .def_property(
            "heel_left_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeelLeftPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> heel_left_position) {
                JPS_HumanoidModelV0State_SetHeelLeftPosition(w.handle, intoJPS_Point(heel_left_position));
            })
        .def_property(
            "heel_left_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetHeelLeftVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> heel_left_velocity) {
                JPS_HumanoidModelV0State_SetHeelLeftVelocity(w.handle, intoJPS_Point(heel_left_velocity));
            })
            ;
}
