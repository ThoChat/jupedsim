// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"
#include "Point3D.hpp"
#include <Eigen/Dense>

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
    // Humanoid model parameters
    double height{}; // agents height
    // Humanoid model variables
    // # gait variables
    int step_duration{}; // total number of time steps required to complete the current step
    int step_timer{};  // number of time step before completing the current step
    int stepping_foot_index{};  //  -1 == right foot stepping/left foot support,
                                //  0 == double support
                                //  1 == left foot stepping/right foot support
    Point step_target{};  // target position of the current stepping foot
    // # body motion variables
    Point3D head_position{}; 
    Point3D pelvis_position{}; 
    double shoulder_rotation_angle_z{}; 
    double trunk_rotation_angle_x{}; 
    double trunk_rotation_angle_y{}; 
    Point3D heel_right_position{}; 
    Point3D heel_left_position{}; 

};

template <>
struct fmt::formatter<HumanoidModelV0Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const HumanoidModelV0Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "SFM([velocity={}, m={}, v0={}, tau={}, A_ped={}, A_obst={}, B={}, r={}, step_duration={}, step_timer={}, stepping_foot_index{}, step_target={}, head_position={}, shoulder_rotation_angle_z={}, trunk_rotation_angle_x={}, trunk_rotation_angle_y={}, heel_right_position={}, heel_left_position={} ])",
            m.velocity,
            m.mass,
            m.desiredSpeed,
            m.reactionTime,
            m.agentScale,
            m.obstacleScale,
            m.forceDistance,
            m.radius,
            m.height,
            m.step_duration,
            m.step_timer,
            m.stepping_foot_index,
            m.step_target,
            m.head_position,
            m.shoulder_rotation_angle_z,
            m.trunk_rotation_angle_x,
            m.trunk_rotation_angle_y,
            m.heel_right_position,
            m.heel_left_position
                        );
    }
};
