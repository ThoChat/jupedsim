// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"
struct HumanoidModelV0Data {
    // SFM parameter, for navigation
    Point velocity{}; // v
    double mass{}; // m
    double desiredSpeed{}; // v0
    double reactionTime{}; // tau
    double agentScale{}; // A for other agents
    double obstacleScale{}; // A for obstacles
    double forceDistance{}; // B
    double radius{}; // r
    // Humanoid model variables
    Point head_position{}; 
    Point head_velocity{}; 
    Point shoulder_right_position{}; 
    Point shoulder_right_velocity{}; 
    Point shoulder_left_position{}; 
    Point shoulder_left_velocity{}; 
};

template <>
struct fmt::formatter<HumanoidModelV0Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const HumanoidModelV0Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "SFM([velocity={}, m={}, v0={}, tau={}, A_ped={}, A_obst={}, B={}, r={}, head_position={}, head_velocity={}, shoulder_right_position={}, shoulder_right_velocity={}, shoulder_left_position={}, shoulder_left_velocity={} ])",
            m.velocity,
            m.mass,
            m.desiredSpeed,
            m.reactionTime,
            m.agentScale,
            m.obstacleScale,
            m.forceDistance,
            m.radius,
            m.head_position,
            m.head_velocity,
            m.shoulder_right_position,
            m.shoulder_right_velocity,
            m.shoulder_left_position,
            m.shoulder_left_velocity
            );
    }
};
