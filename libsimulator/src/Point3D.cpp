// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "Point3D.hpp"

#include "Macros.hpp"

#include <Logger.hpp>
#include <limits>

double Point3D::Norm() const
{
    return sqrt(NormSquare());
}

Point3D Point3D::Normalized() const
{
    return std::get<1>(NormAndNormalized());
}

std::tuple<double, Point3D> Point3D::NormAndNormalized() const
{
    const double norm = Norm();
    if(norm > std::numeric_limits<double>::epsilon())
        return std::make_tuple(norm, (Point3D(x, y,z) / norm));
    else
        return std::make_tuple(0.0, Point3D(0.0, 0.0, 0.0));
}


bool Point3D::IsUnitLength() const
{
    return std::abs(1 - NormSquare()) <= std::numeric_limits<double>::epsilon();
}

const Point3D Point3D::operator+(const Point3D& p) const
{
    return Point3D(x + p.x, y + p.y, z + p.z);
}

const Point3D Point3D::operator-(const Point3D& p) const
{
    return Point3D(x - p.x, y - p.y, z - p.z);
}

bool Point3D::operator==(const Point3D& p) const
{
    return x == p.x && y == p.y && z == p.z;
}

bool Point3D::operator!=(const Point3D& p) const
{
    return !(*this == p);
}

const Point3D operator*(const Point3D& p, double f)
{
    return Point3D(p.x * f, p.y * f, p.z * f);
}

Point3D& Point3D::operator+=(const Point3D& p)
{
    x += p.x;
    y += p.y;
    z += p.z;
    return *this;
}

const Point3D operator/(const Point3D& p, double f)
{
    static auto constexpr eps =
        std::numeric_limits<double>::epsilon() * std::numeric_limits<double>::epsilon();
    if(f > eps)
        return Point3D(p.x / f, p.y / f, p.z / f);
    else {
        LOG_WARNING("Point3D::operator/ dividend {} is too small. Using 1 instead.", f);
        return Point3D(p.x, p.y, p.z);
    }
}

bool Point3D::operator<(const Point3D& rhs) const
{
    if(x < rhs.x)
        return true;
    else if((x == rhs.x) && (y < rhs.y) && (z < rhs.z))
        return true;
    return false;
}

bool Point3D::operator>(const Point3D& rhs) const
{
    return rhs < *this;
}

bool Point3D::operator<=(const Point3D& rhs) const
{
    return !(rhs < *this);
}

bool Point3D::operator>=(const Point3D& rhs) const
{
    return !(*this < rhs);
}

double Distance(const Point3D& point1, const Point3D& point2)
{
    return (point1 - point2).Norm();
}

double DistanceSquared(const Point3D& a, const Point3D& b)
{
    return (a - b).NormSquare();
}
