// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <fmt/format.h>

#include <cmath>
#include <iostream>
#include <string>

class Point3D
{
public:
    double x{};
    double y{};
    double z{};
public:
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {};

    /// Norm
    double Norm() const;

    /// Norm square
    inline double NormSquare() const { return ScalarProduct(*this); }

    /// normalized vector
    Point3D Normalized() const;

    /// Return norm and direction in one call
    /// @return Norm and Normalized
    std::tuple<double, Point3D> NormAndNormalized() const;

    /// dot product
    inline double ScalarProduct(const Point3D& v) const { return x * v.x + y * v.y; }

    inline double CrossProduct(const Point3D& p) const { return Determinant(p); }

    /// determinant of the square matrix formed by the vectors [ this, v]
    inline double Determinant(const Point3D& v) const { return x * v.y - y * v.x; }

    Point3D TransformToEllipseCoordinates(const Point3D& center, double cphi, double sphi) const;
    /// translation and rotation in cartesian system
    Point3D TransformToCartesianCoordinates(const Point3D& center, double cphi, double sphi) const;
    /// rotate the vector by theta
    Point3D Rotate(double ctheta, double stheta) const;

    /// Create a new vector rotated by +90 degree (ccw rotation)
    /// @return rotated vector
    Point3D Rotate90Deg() const;

    /// Tests that the vector is length 1
    /// @return length == 1
    bool IsUnitLength() const;

    // operators
    /// addition
    const Point3D operator+(const Point3D& p) const;
    /// substraction
    const Point3D operator-(const Point3D& p) const;
    /// equal
    bool operator==(const Point3D& p) const;
    /// not equal
    bool operator!=(const Point3D& p) const;
    /// Assignement
    Point3D& operator+=(const Point3D& p);

    bool operator<(const Point3D& rhs) const;

    bool operator>(const Point3D& rhs) const;

    bool operator<=(const Point3D& rhs) const;

    bool operator>=(const Point3D& rhs) const;
};

/// Euclidean distance between 'a' and 'b'
/// @param [in] Point3D a
/// @param [in] Point3D b
/// @return distance between 'a' and 'b'
double Distance(const Point3D& a, const Point3D& b);

/// Squared euclidean distance between 'a' and 'b'
/// @param [in] Point3D a
/// @param [in] Point3D b
/// @return distance between 'a' and 'b'
double DistanceSquared(const Point3D& a, const Point3D& b);

/// multiplication
const Point3D operator*(const Point3D& p, const double f);
/// division
const Point3D operator/(const Point3D& p, const double f);

template <>
struct fmt::formatter<Point3D> {
    char presentation{'f'};

    constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const Point3D& p, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "({}, {})", p.x, p.y);
    }
};
