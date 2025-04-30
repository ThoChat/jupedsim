// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once
#include <jupedsim/jupedsim.h>

#include <tuple>
#include <vector>

// handeling 2D points
std::tuple<double, double> intoTuple(const JPS_Point& p);

std::vector<std::tuple<double, double>> intoTuple(const std::vector<JPS_Point>& p);

std::vector<std::tuple<double, double>> intoTuple(const JPS_Point* beg, const JPS_Point* end);

JPS_Point intoJPS_Point(const std::tuple<double, double> p);

std::vector<JPS_Point> intoJPS_Point(const std::vector<std::tuple<double, double>>& p);

// handeling 3D points
std::tuple<double, double, double> intoTuple3D(const JPS_Point3D& p);

std::vector<std::tuple<double, double, double>> intoTuple3D(const std::vector<JPS_Point3D>& p);

std::vector<std::tuple<double, double, double>> intoTuple3D(const JPS_Point3D* beg, const JPS_Point3D* end);

JPS_Point3D intoJPS_Point3D(const std::tuple<double, double, double> p);

std::vector<JPS_Point3D> intoJPS_Point3D(const std::vector<std::tuple<double, double, double>>& p);