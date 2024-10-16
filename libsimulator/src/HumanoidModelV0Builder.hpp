// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "HumanoidModelV0.hpp"
class HumanoidModelV0Builder
{
    double _bodyForce;
    double _friction;

public:
    HumanoidModelV0Builder(double bodyForce, double friction);
    HumanoidModelV0 Build();
};
