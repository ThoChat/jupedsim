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
        return std::make_tuple(norm, (Point3D(x, y) / norm));
    else
        return std::make_tuple(0.0, Point3D(0.0, 0.0));
}

/* Transformiert die "normalen" Koordinaten in Koordinaten der Ellipse
 * dazu verschieben des Koordinaten Ursprungs in Center und anschliessend drehen um phi
 * alle Punkte müssen in "normalen" Koordinaten gegeben sein
 * center: Center der Ellipse in deren System transformiert werden soll
 * phi: Winkel der Ellipse in deren System transformiert werden soll
 * */

/*coordinate transformation of the point P(x,y) expressed in coord system S1 to a new coord. sys S2

           A
           *
         |     S_2
     \   |   /
 |    \  |  /
 |     \ | /^phi
 | yc___\ /_)_________ S_3
 |       O1
 |       |
 |       |
 |       xc
 |
 |___________________________
O
S_1


////////////////////////////////////
S_1 is cartesian coordinate system!!
////////////////////////////////////

  input:
  - (x,y)        :  coordinates of the point A in S_1
  - (xc,yc)      : coordinate of the center in the  S_1 (Center of Ellipse)
  - phi          : angle between the S_1 and S_2

  output:
  +  (xnew,ynew) : new coordinate of the point A in the coord. sys S2

OA = OO1 + O1A

 [x ; y] = [xc ; yc] +  [x_3 ; y_3]   : (1) ( with [x_i ; y_i] coordinats of P in S_i and i in
{1,2,3} )

[x_2 ; y_2] = M(phi) * [x_3 ; y_3]  : (2)


(1) in (2)--->

-->  [x_2 ; y_2] = M(phi) * ([x ; y] - [xc ; yc])



after rotation:
OC = OO1 +O1C
OC  = -O1O +O1C

xnew = -xc + x

*/
Point3D Point3D::TransformToEllipseCoordinates(const Point3D& center, double cphi, double sphi) const
{
    Point3D p = Point3D(x, y);
    return (p - center).Rotate(cphi, -sphi);
}

/*
This is the reverse funktion of TransformToEllipseCoordinates(),
where the coord. of a point are transformated to cart. coord.

 input:
  - (x,y)        :  coordinates of the point P in S_2
  - (xc,yc)      : coordinate of the center in the  S_1 (Center of Ellipse)
  - phi          : angle between the S_1 and S_2

  output:
  +  (xnew,ynew) : new coordinate of the point P in the coord. sys S_1

[x_2 ; y_2] = M(phi) * ([x ; y] - [xc ; yc]) (see comments in CoordTransToEllipse() )


----> [x ; y] =  M(-phi) * [x_2 ; y_2] +  [xc ; yc]

*/

Point3D Point3D::TransformToCartesianCoordinates(const Point3D& center, double cphi, double sphi) const
{
    Point3D p = Point3D(x, y);
    return (p.Rotate(cphi, sphi) + center);
}

/*rotate a two-dimensional vector by an angle of theta

Rotation-matrix=[cos(theta)  -sin(theta)]
                [ sin(theta)  cos(theta)]

*/
Point3D Point3D::Rotate(double ctheta, double stheta) const
{
    return Point3D(x * ctheta - y * stheta, x * stheta + y * ctheta);
}

Point3D Point3D::Rotate90Deg() const
{
    return {-y, x};
}

bool Point3D::IsUnitLength() const
{
    return std::abs(1 - NormSquare()) <= std::numeric_limits<double>::epsilon();
}

const Point3D Point3D::operator+(const Point3D& p) const
{
    return Point3D(x + p.x, y + p.y);
}

const Point3D Point3D::operator-(const Point3D& p) const
{
    return Point3D(x - p.x, y - p.y);
}

bool Point3D::operator==(const Point3D& p) const
{
    return x == p.x && y == p.y;
}

bool Point3D::operator!=(const Point3D& p) const
{
    return !(*this == p);
}

const Point3D operator*(const Point3D& p, double f)
{
    return Point3D(p.x * f, p.y * f);
}

Point3D& Point3D::operator+=(const Point3D& p)
{
    x += p.x;
    y += p.y;
    return *this;
}

const Point3D operator/(const Point3D& p, double f)
{
    static auto constexpr eps =
        std::numeric_limits<double>::epsilon() * std::numeric_limits<double>::epsilon();
    if(f > eps)
        return Point3D(p.x / f, p.y / f);
    else {
        LOG_WARNING("Point3D::operator/ dividend {} is too small. Using 1 instead.", f);
        return Point3D(p.x, p.y);
    }
}

bool Point3D::operator<(const Point3D& rhs) const
{
    if(x < rhs.x)
        return true;
    else if((x == rhs.x) && (y < rhs.y))
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
