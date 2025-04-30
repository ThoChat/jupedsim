// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "Conversion.hpp"

namespace jupedsim::detail
{
// handeling 2D points
Point intoPoint(JPS_Point p)
{
    return {p.x, p.y};
}

JPS_Point intoJPS_Point(Point p)
{
    return {p.x, p.y};
}

JPS_Point intoJPS_Point(std::tuple<double, double> p)
{
    return {std::get<0>(p), std::get<1>(p)};
}

std::tuple<double, double> intoTuple(JPS_Point p)
{
    return std::make_tuple(p.x, p.y);
}

// handeling 3D points
Point3D intoPoint3D(JPS_Point3D p)
{
    return {p.x, p.y, p.z}; 

} 

JPS_Point3D intoJPS_Point3D(Point3D p)
{
    return {p.x, p.y, p.z};
}

JPS_Point3D intoJPS_Point3D(std::tuple<double, double, double> p)
{
    return {std::get<0>(p), std::get<1>(p), std::get<2>(p)};
}

std::tuple<double, double, double> intoTuple3D(JPS_Point3D p)
{
    return std::make_tuple(p.x, p.y, p.z);
}

} // namespace jupedsim::detail
