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
                        // variables of the humanoid body
                        std::tuple<double, double> head_position, 
                        std::tuple<double, double> head_velocity,
                        std::tuple<double, double> shoulder_right_position, 
                        std::tuple<double, double> shoulder_right_velocity,
                        std::tuple<double, double> shoulder_left_position, 
                        std::tuple<double, double> shoulder_left_velocity,
                        std::tuple<double, double> pelvis_right_position, 
                        std::tuple<double, double> pelvis_right_velocity,
                        std::tuple<double, double> pelvis_left_position, 
                        std::tuple<double, double> pelvis_left_velocity,
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
                    intoJPS_Point(head_position),
                    intoJPS_Point(head_velocity),
                    intoJPS_Point(shoulder_right_position),
                    intoJPS_Point(shoulder_right_velocity),
                    intoJPS_Point(shoulder_left_position),
                    intoJPS_Point(shoulder_left_velocity),
                    intoJPS_Point(pelvis_right_position),
                    intoJPS_Point(pelvis_right_velocity),
                    intoJPS_Point(pelvis_left_position),
                    intoJPS_Point(pelvis_left_velocity),
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
            py::arg("head_position"),
            py::arg("head_velocity"),
            py::arg("shoulder_right_position"),
            py::arg("shoulder_right_velocity"),
            py::arg("shoulder_left_position"),
            py::arg("shoulder_left_velocity"),
            py::arg("pelvis_right_position"),
            py::arg("pelvis_right_velocity"),
            py::arg("pelvis_left_position"),
            py::arg("pelvis_left_velocity"),
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
                "radius: {}, head_position: {}, head_velocity: {}, shoulder_right_position: {},"
                "shoulder_right_velocity: {}, shoulder_left_position: {}, shoulder_left_velocity: {},"
                "pelvis_right_position: {}, pelvis_right_velocity: {}, pelvis_left_position: {}, pelvis_left_velocity: {},"
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
                intoTuple(p.head_position),
                intoTuple(p.head_velocity),
                intoTuple(p.shoulder_right_position),
                intoTuple(p.shoulder_right_velocity),
                intoTuple(p.shoulder_left_position),
                intoTuple(p.shoulder_left_velocity),
                intoTuple(p.pelvis_right_position),
                intoTuple(p.pelvis_right_velocity),
                intoTuple(p.pelvis_left_position),
                intoTuple(p.pelvis_left_velocity),
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
            "shoulder_right_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetShoulderRightPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> shoulder_right_position) {
                JPS_HumanoidModelV0State_SetShoulderRightPosition(w.handle, intoJPS_Point(shoulder_right_position));
            })
        .def_property(
            "shoulder_right_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetShoulderRightVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> shoulder_right_velocity) {
                JPS_HumanoidModelV0State_SetShoulderRightVelocity(w.handle, intoJPS_Point(shoulder_right_velocity));
            })
        .def_property(
            "shoulder_left_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetShoulderLeftPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> shoulder_left_position) {
                JPS_HumanoidModelV0State_SetShoulderLeftPosition(w.handle, intoJPS_Point(shoulder_left_position));
            })
        .def_property(
            "shoulder_left_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetShoulderLeftVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> shoulder_left_velocity) {
                JPS_HumanoidModelV0State_SetShoulderLeftVelocity(w.handle, intoJPS_Point(shoulder_left_velocity));
            })
        .def_property(
            "pelvis_right_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetPelvisRightPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> pelvis_right_position) {
                JPS_HumanoidModelV0State_SetPelvisRightPosition(w.handle, intoJPS_Point(pelvis_right_position));
            })
        .def_property(
            "pelvis_right_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetPelvisRightVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> pelvis_right_velocity) {
                JPS_HumanoidModelV0State_SetPelvisRightVelocity(w.handle, intoJPS_Point(pelvis_right_velocity));
            })
        .def_property(
            "pelvis_left_position",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetPelvisLeftPosition(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> pelvis_left_position) {
                JPS_HumanoidModelV0State_SetPelvisLeftPosition(w.handle, intoJPS_Point(pelvis_left_position));
            })
        .def_property(
            "pelvis_left_velocity",
            [](const JPS_HumanoidModelV0State_Wrapper& w) {
                return intoTuple(JPS_HumanoidModelV0State_GetPelvisLeftVelocity(w.handle));
            },
            [](JPS_HumanoidModelV0State_Wrapper& w, std::tuple<double, double> pelvis_left_velocity) {
                JPS_HumanoidModelV0State_SetPelvisLeftVelocity(w.handle, intoJPS_Point(pelvis_left_velocity));
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
