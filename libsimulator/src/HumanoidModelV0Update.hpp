// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point.hpp"
#include "Point3D.hpp"

#include <optional>

struct HumanoidModelV0Update {
    Point position{};
    Point velocity{};
    // Humanoid model variables
    // # gait variables
    int step_timer{}; 
    int stepping_foot_index{}; 
    Point step_target{};
    // # body motion variables
    // head
    Point3D head_position{};
    Point head_velocity{};
    // shoulder rotation
    double shoulder_rotation_angle_z{};
    double shoulder_rotation_velocity_z{};
    // trunk 
    double trunk_rotation_angle_x{};
    double trunk_rotation_velocity_x{};
    double trunk_rotation_angle_y{};
    double trunk_rotation_velocity_y{};
    // heels
    Point heel_right_position{};
    Point heel_right_velocity{};
    Point heel_left_position{};
    Point heel_left_velocity{};
    
};
