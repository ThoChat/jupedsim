// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "Point.hpp"
#include "Point3D.hpp"
#include "jupedsim/jupedsim.h"
#include <tuple>

namespace jupedsim::detail
{
// handeling 2D points
Point intoPoint(JPS_Point p);

JPS_Point intoJPS_Point(Point p);

JPS_Point intoJPS_Point(std::tuple<double, double> p);

std::tuple<double, double> intoTuple(JPS_Point p);

// handeling 3D points
Point3D intoPoint3D(JPS_Point3D p);

JPS_Point3D intoJPS_Point3D(Point3D p);

JPS_Point3D intoJPS_Point3D(std::tuple<double, double, double> p);

std::tuple<double, double, double> intoTuple3D(JPS_Point3D p);
} // namespace jupedsim::detail
