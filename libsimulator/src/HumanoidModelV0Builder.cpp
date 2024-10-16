// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "HumanoidModelV0Builder.hpp"
#include "HumanoidModelV0.hpp"

HumanoidModelV0Builder::HumanoidModelV0Builder(double bodyForce, double friction)
    : _bodyForce(bodyForce), _friction(friction)
{
}

HumanoidModelV0 HumanoidModelV0Builder::Build()
{
    return HumanoidModelV0(_bodyForce, _friction);
}
