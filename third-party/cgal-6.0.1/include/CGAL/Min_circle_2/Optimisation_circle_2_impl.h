// Copyright (c) 1997-2001
// ETH Zurich (Switzerland).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Bounding_volumes/include/CGAL/Min_circle_2/Optimisation_circle_2_impl.h $
// $Id: include/CGAL/Min_circle_2/Optimisation_circle_2_impl.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Sven Schoenherr <sven@inf.ethz.ch>, Bernd Gaertner

#ifndef CGAL_MIN_SPHERE_D_OPTIMISATION_CIRCLE_2_IMPL_H
#define CGAL_MIN_SPHERE_D_OPTIMISATION_CIRCLE_2_IMPL_H

#include <CGAL/license/Bounding_volumes.h>

// includes
#  include <CGAL/assertions.h>

namespace CGAL {

// Class implementation (continued)
// ================================

// I/O
// ---
template < class K_ >
std::ostream&
operator << ( std::ostream& os, const CGAL::Optimisation_circle_2<K_>& c)
{
    switch ( CGAL::IO::get_mode( os)) {

      case CGAL::IO::PRETTY:
        os << "CGAL::Optimisation_circle_2( "
           << c.center() << ", "
           << c.squared_radius() << ')';
        break;

      case CGAL::IO::ASCII:
        os << c.center() << ' ' << c.squared_radius();
        break;

      case CGAL::IO::BINARY:
        os << c.center();
        CGAL::write( os, c.squared_radius());
        break;

      default:
        CGAL_assertion_msg( false,
                                         "CGAL::IO::get_mode( os) invalid!");
        break; }

    return( os);
}

template < class K_ >
std::istream&
operator >> ( std::istream& is, CGAL::Optimisation_circle_2<K_>& c)
{
    typedef  typename CGAL::Optimisation_circle_2<K_>::Point     Point;
    typedef  typename CGAL::Optimisation_circle_2<K_>::Distance  Distance;

    switch ( CGAL::IO::get_mode( is)) {

      case CGAL::IO::PRETTY:
        std::cerr << std::endl;
        std::cerr << "Stream must be in ASCII or binary mode" << std::endl;
        break;

      case CGAL::IO::ASCII: {
        Point     center;
        Distance  squared_radius;
        is >> center >> squared_radius;
        c.set( center, squared_radius); }
        break;

      case CGAL::IO::BINARY: {
        Point     center;
        Distance  squared_radius;
        is >> center;
        CGAL::read( is, squared_radius);
        c.set( center, squared_radius); }
        break;

      default:
        CGAL_assertion_msg( false,
                                         "CGAL::IO::get_mode( is) invalid!");
        break; }

    return( is);
}

} //namespace CGAL

// ===== EOF ==================================================================

#endif //CGAL_MIN_SPHERE_D_OPTIMISATION_CIRCLE_2_IMPL_H
