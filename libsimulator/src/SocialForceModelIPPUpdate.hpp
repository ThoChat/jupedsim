// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Point.hpp"

struct SocialForceModelIPPUpdate {
    Point position{};
    Point ground_support_position{};
    Point ground_support_velocity{};
    Point velocity{};
};
