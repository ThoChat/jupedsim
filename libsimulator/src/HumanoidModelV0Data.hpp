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
    int step_timer{};  // number of time step before completing the current step
    int stepping_foot_index{};  //  -1 == right foot stepping/left foot support,
                                //  0 == double support
                                //  1 == left foot stepping/right foot support
    Point step_target{};  // target position of the current stepping foot
    // # body motion variables
    Point3D head_position{}; 
    Point head_velocity{}; 
    double shoulder_rotation_angle_z{}; 
    double shoulder_rotation_velocity_z{}; 
    double trunk_rotation_angle_x{}; 
    double trunk_rotation_velocity_x{}; 
    double trunk_rotation_angle_y{}; 
    double trunk_rotation_velocity_y{}; 
    Point3D heel_right_position{}; 
    Point heel_right_velocity{}; 
    Point3D heel_left_position{}; 
    Point heel_left_velocity{}; 
    Eigen::MatrixXd joint_angles_matrix {};     // Rows: x/y/z rotation, Columns: joints * 11
    Eigen::MatrixXd joint_position_matrix {};   // Rows: x/y/z/1 position (for Denavit Hartenberg tranform)
                                                // , Columns: joints *11
                                                // the referencial of the coordinate in linked to the pelvis so that
                                                // x == sagittal, y == frontal, z == vertical (up) axis
    // # body parameters                        
    // list of all simulated joitns
    /**
    0 - right heel
    1 - right ankle
    2 - right hip
    3 - left hip
    4 - left ankle
    5 - left heel
    6 - pelvis/CoM
    7 - right shoulder
    8 - C7 / neck
    9 - left shoulder
    10 - head 
     */

};

template <>
struct fmt::formatter<HumanoidModelV0Data> {

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const HumanoidModelV0Data& m, FormatContext& ctx) const
    {
        return fmt::format_to(
            ctx.out(),
            "SFM([velocity={}, m={}, v0={}, tau={}, A_ped={}, A_obst={}, B={}, r={}, step_timer={}, stepping_foot_index{}, step_target={}, head_position={}, head_velocity={}, shoulder_rotation_angle_z={}, shoulder_rotation_velocity_z={}, trunk_rotation_angle_x={}, trunk_rotation_velocity_x={}, trunk_rotation_angle_y={}, trunk_rotation_velocity_y={}, heel_right_position={}, heel_right_velocity={}, heel_left_position={}, heel_left_velocity={}, joint_angles_matrix={}, joint_position_matrix={} ])",
            m.velocity,
            m.mass,
            m.desiredSpeed,
            m.reactionTime,
            m.agentScale,
            m.obstacleScale,
            m.forceDistance,
            m.radius,
            m.height,
            m.step_timer,
            m.stepping_foot_index,
            m.step_target,
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
            m.heel_left_velocity,
            m.joint_angles_matrix,
            m.joint_position_matrix
                        );
    }
};
