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
    double shoulder_rotation_angle_z{}; 
    double shoulder_rotation_velocity_z{}; 
    double trunk_rotation_angle_x{}; 
    double trunk_rotation_velocity_x{}; 
    double trunk_rotation_angle_y{}; 
    double trunk_rotation_velocity_y{}; 
    Point heel_right_position{}; 
    Point heel_right_velocity{}; 
    Point heel_left_position{}; 
    Point heel_left_velocity{}; 


};

template <>
struct fmt::formatter<HumanoidModelV0Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const HumanoidModelV0Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "SFM([velocity={}, m={}, v0={}, tau={}, A_ped={}, A_obst={}, B={}, r={}, head_position={}, head_velocity={}, shoulder_rotation_angle_z={}, shoulder_rotation_velocity_z={}, trunk_rotation_angle_x={}, trunk_rotation_velocity_x={}, trunk_rotation_angle_y={}, trunk_rotation_velocity_y={}, heel_right_position={}, heel_right_velocity={}, heel_left_position={}, heel_left_velocity={} ])",
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
            m.shoulder_rotation_angle_z,
            m.shoulder_rotation_velocity_z,
            m.trunk_rotation_angle_x,
            m.trunk_rotation_velocity_x,
            m.trunk_rotation_angle_y,
            m.trunk_rotation_velocity_y,  
            m.heel_right_position,
            m.heel_right_velocity,
            m.heel_left_position,
            m.heel_left_velocity

            );
    }
};
