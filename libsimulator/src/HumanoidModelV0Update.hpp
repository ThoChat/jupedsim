// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point.hpp"

#include <optional>

struct HumanoidModelV0Update {
    Point position{};
    Point velocity{};
    // Humanoid model variables
    Point head_position{}; // h_p
    Point head_velocity{}; // h_v
};
