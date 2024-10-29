// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point.hpp"

#include <optional>

struct HumanoidModelV0Update {
    Point position{};
    Point velocity{};
    // Humanoid model variables
    // head
    Point head_position{};
    Point head_velocity{};
    // shoulders
    Point shoulder_right_position{};
    Point shoulder_right_velocity{};
    Point shoulder_left_position{};
    Point shoulder_left_velocity{};
    // pelvis
    Point pelvis_right_position{};
    Point pelvis_right_velocity{};
    Point pelvis_left_position{};
    Point pelvis_left_velocity{};
    // heels
    Point heel_right_position{};
    Point heel_right_velocity{};
    Point heel_left_position{};
    Point heel_left_velocity{};
};
