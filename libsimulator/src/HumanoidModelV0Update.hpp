// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point.hpp"
#include "Point3D.hpp"
#include <Eigen/Dense>
#include <optional>

struct HumanoidModelV0Update {
    Point position{};
    Point velocity{};
    // Humanoid model variables
    // # gait variables
    int step_duration{};
    int step_timer{}; 
    int stepping_foot_index{}; 
    Point step_target{};
    // # body motion variables
    // head
    Point3D head_position{};
    // pelvis
    Point3D pelvis_position{};
    double pelvis_rotation_angle_z{};
    // shoulder rotation
    double shoulder_rotation_angle_z{};
    // trunk rotation
    double trunk_rotation_angle_x{};
    double trunk_rotation_angle_y{};
    // heels
    Point3D heel_right_position{};
    Point3D heel_left_position{};
    // Toes
    Point3D toe_right_position{};
    Point3D toe_left_position{};

    
};
