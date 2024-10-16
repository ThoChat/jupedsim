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
                        double radius) {
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
                    radius};
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
            py::arg("radius"))
        .def("__repr__", [](const JPS_HumanoidModelV0AgentParameters& p) {
            return fmt::format(
                "position: {}, orientation: {}, journey_id: {}, stage_id: {},"
                "velocity: {}, mass: {}, desiredSpeed: {},"
                "reactionTime: {}, agentScale: {}, obstacleScale: {}, forceDistance: {}, radius: "
                "{}",
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
                p.radius);
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
            });
}
