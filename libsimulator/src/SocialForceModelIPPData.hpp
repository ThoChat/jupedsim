// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

#include <fmt/core.h>

struct SocialForceModelIPPData {
    Point velocity{}; // v
    Point ground_support_position{}; // position of ground support circle
    Point ground_support_velocity{}; // velocity of ground support circle
    double height{1.75}; // height of the agent [m]
    double desiredSpeed{1.2}; // v0 [m/s]
    double reactionTime{0.5}; // tau [s]
    double lambdaU{0.5}; // unbalancing rate [1/s]
    double lambdaB{1.0}; // balancing rate [1/s]
    double balanceSpeed{1.0}; // v_s coupling speed [m/s]
    double damping{1.0}; // lambda velocity dissipation [1/s]
    double agentScale{5.0}; // A repulsion amplitude vs agents [N]
    double obstacleScale{5.0}; // A_w repulsion amplitude vs walls [N]
    double forceDistance{0.5}; // B upper body interaction range [m]
    double obstacleForceDistance{0.3}; // B_w wall interaction range [m]
    double legForceDistance{0.3}; // B_leg leg interaction range [m]
    double radius{0.15}; // r upper body radius [m]
};

template <>
struct fmt::formatter<SocialForceModelIPPData> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const SocialForceModelIPPData& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "IPP[velocity={}, gs_pos={}, gs_vel={}, h={}, v0={}, tau={}, "
            "lambda_u={}, lambda_b={}, v_s={}, lambda={}, A={}, A_w={}, B={}, B_w={}, B_leg={}, "
            "r={}]",
            m.velocity,
            m.ground_support_position,
            m.ground_support_velocity,
            m.height,
            m.desiredSpeed,
            m.reactionTime,
            m.lambdaU,
            m.lambdaB,
            m.balanceSpeed,
            m.damping,
            m.agentScale,
            m.obstacleScale,
            m.forceDistance,
            m.obstacleForceDistance,
            m.legForceDistance,
            m.radius);
    }
};
