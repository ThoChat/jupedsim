// SPDX-License-Identifier: LGPL-3.0-or-later
#include "OperationalModel.hpp"
#include "SocialForceModelIPP.hpp"
#include "SocialForceModelIPPBuilder.hpp"
#include "SocialForceModelIPPData.hpp"
#include "conversion.hpp"

#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep

#include <tuple>

namespace py = pybind11;

void init_social_force_model_IPP(py::module_& m)
{
    py::class_<SocialForceModelIPP, OperationalModel>(m, "SocialForceModelIPP");
    py::class_<SocialForceModelIPPBuilder>(m, "SocialForceModelIPPBuilder")
        .def(py::init<>())
        .def("build", &SocialForceModelIPPBuilder::Build);
    py::class_<SocialForceModelIPPData>(m, "SocialForceModelIPPState")
        .def(
            py::init([](std::tuple<double, double> velocity,
                        std::tuple<double, double> groundSupportPosition,
                        std::tuple<double, double> groundSupportVelocity,
                        double height,
                        double desiredSpeed,
                        double reactionTime,
                        double lambdaU,
                        double lambdaB,
                        double balanceSpeed,
                        double damping,
                        double agentScale,
                        double obstacleScale,
                        double forceDistance,
                        double obstacleForceDistance,
                        double legForceDistance,
                        double radius) {
                return SocialForceModelIPPData{
                    .velocity = intoPoint(velocity),
                    .ground_support_position = intoPoint(groundSupportPosition),
                    .ground_support_velocity = intoPoint(groundSupportVelocity),
                    .height = height,
                    .desiredSpeed = desiredSpeed,
                    .reactionTime = reactionTime,
                    .lambdaU = lambdaU,
                    .lambdaB = lambdaB,
                    .balanceSpeed = balanceSpeed,
                    .damping = damping,
                    .agentScale = agentScale,
                    .obstacleScale = obstacleScale,
                    .forceDistance = forceDistance,
                    .obstacleForceDistance = obstacleForceDistance,
                    .legForceDistance = legForceDistance,
                    .radius = radius};
            }),
            py::kw_only(),
            py::arg("velocity"),
            py::arg("ground_support_position"),
            py::arg("ground_support_velocity"),
            py::arg("height"),
            py::arg("desired_speed"),
            py::arg("reaction_time"),
            py::arg("lambda_u"),
            py::arg("lambda_b"),
            py::arg("balance_speed"),
            py::arg("damping"),
            py::arg("agent_scale"),
            py::arg("obstacle_scale"),
            py::arg("force_distance"),
            py::arg("obstacle_force_distance"),
            py::arg("leg_force_distance"),
            py::arg("radius"))
        .def_property(
            "velocity",
            [](const SocialForceModelIPPData& obj) { return intoTuple(obj.velocity); },
            [](SocialForceModelIPPData& obj, std::tuple<double, double> pt) {
                obj.velocity = intoPoint(pt);
            })
        .def_property(
            "ground_support_position",
            [](const SocialForceModelIPPData& obj) {
                return intoTuple(obj.ground_support_position);
            },
            [](SocialForceModelIPPData& obj, std::tuple<double, double> pt) {
                obj.ground_support_position = intoPoint(pt);
            })
        .def_property(
            "ground_support_velocity",
            [](const SocialForceModelIPPData& obj) {
                return intoTuple(obj.ground_support_velocity);
            },
            [](SocialForceModelIPPData& obj, std::tuple<double, double> pt) {
                obj.ground_support_velocity = intoPoint(pt);
            })
        .def_readwrite("height", &SocialForceModelIPPData::height)
        .def_readwrite("desired_speed", &SocialForceModelIPPData::desiredSpeed)
        .def_readwrite("reaction_time", &SocialForceModelIPPData::reactionTime)
        .def_readwrite("lambda_u", &SocialForceModelIPPData::lambdaU)
        .def_readwrite("lambda_b", &SocialForceModelIPPData::lambdaB)
        .def_readwrite("balance_speed", &SocialForceModelIPPData::balanceSpeed)
        .def_readwrite("damping", &SocialForceModelIPPData::damping)
        .def_readwrite("agent_scale", &SocialForceModelIPPData::agentScale)
        .def_readwrite("obstacle_scale", &SocialForceModelIPPData::obstacleScale)
        .def_readwrite("force_distance", &SocialForceModelIPPData::forceDistance)
        .def_readwrite("obstacle_force_distance", &SocialForceModelIPPData::obstacleForceDistance)
        .def_readwrite("leg_force_distance", &SocialForceModelIPPData::legForceDistance)
        .def_readwrite("radius", &SocialForceModelIPPData::radius);
}
