// Copyright (c) 2017  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Periodic_3_triangulation_3/include/CGAL/Periodic_3_triangulation_3/internal/Periodic_3_regular_triangulation_statically_filtered_traits_3.h $
// $Id: include/CGAL/Periodic_3_triangulation_3/internal/Periodic_3_regular_triangulation_statically_filtered_traits_3.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Mael Rouxel-Labbé

#ifndef CGAL_PERIODIC_3_REGULAR_TRIANGULATION_STATICALLY_FILTERED_TRAITS_3_H
#define CGAL_PERIODIC_3_REGULAR_TRIANGULATION_STATICALLY_FILTERED_TRAITS_3_H

#include <CGAL/license/Periodic_3_triangulation_3.h>

#include <CGAL/Periodic_3_triangulation_3/internal/Static_filters/Periodic_3_power_side_of_oriented_power_sphere_3.h>
#include <CGAL/Periodic_3_triangulation_3/internal/Periodic_3_regular_triangulation_filtered_traits_3.h>

namespace CGAL {

template<typename K_,
         typename Off_ = typename CGAL::Periodic_3_offset_3>
class Periodic_3_regular_triangulation_statically_filtered_traits_3
  : public Periodic_3_regular_triangulation_filtered_traits_base_3<K_, Off_>
{
  typedef Periodic_3_regular_triangulation_statically_filtered_traits_3<K_, Off_> Self;
  typedef Periodic_3_regular_triangulation_filtered_traits_base_3<K_, Off_>       Base;

public:
  typedef K_                                                                      Kernel;
  typedef typename Kernel::Iso_cuboid_3                                           Iso_cuboid_3;

  Periodic_3_regular_triangulation_statically_filtered_traits_3(const Iso_cuboid_3& domain,
                                                                const Kernel& k)
    : Base(domain, k)
  { }

#if 0 // @todo
  typedef internal::Static_filters_predicates::
            Periodic_3_power_side_of_oriented_power_sphere_3<
              Self, typename Base::Periodic_3_power_side_of_oriented_power_sphere_3>
            Power_side_of_oriented_power_sphere_3;

  Power_side_of_oriented_power_sphere_3
  power_side_of_oriented_power_sphere_3_object() const
  {
    return Power_side_of_oriented_power_sphere_3(&this->_domain,
                                                 &this->_domain_e,
                                                 &this->_domain_f);
  }
#endif
};

} //namespace CGAL

#endif // CGAL_PERIODIC_3_REGULAR_TRIANGULATION_STATICALLY_FILTERED_TRAITS_3_H
