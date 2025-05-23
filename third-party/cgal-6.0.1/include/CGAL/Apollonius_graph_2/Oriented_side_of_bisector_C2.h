// Copyright (c) 2003,2004  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Apollonius_graph_2/include/CGAL/Apollonius_graph_2/Oriented_side_of_bisector_C2.h $
// $Id: include/CGAL/Apollonius_graph_2/Oriented_side_of_bisector_C2.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>



#ifndef CGAL_APOLLONIUS_GRAPH_2_ORIENTED_SIDE_OF_BISECTOR_C2_H
#define CGAL_APOLLONIUS_GRAPH_2_ORIENTED_SIDE_OF_BISECTOR_C2_H

#include <CGAL/license/Apollonius_graph_2.h>


#include <CGAL/Apollonius_graph_2/basic.h>
#include <CGAL/functions_on_signs.h>


namespace CGAL {

namespace ApolloniusGraph_2 {

//--------------------------------------------------------------------

template<class K, class MTag>
class Oriented_side_of_bisector_2
{
public:
  typedef K                               Kernel;
  typedef MTag                            Method_tag;

  typedef typename K::Point_2             Point_2;
  typedef typename K::Site_2              Site_2;
  typedef typename K::Comparison_result   Comparison_result;
  typedef typename K::Sign                Sign;
  typedef typename K::Oriented_side       Oriented_side;

private:
  typedef typename Kernel::RT             RT;

private:
  Comparison_result
  compare_distances(const Site_2& p1, const Site_2& p2,
                    const Point_2& p, const Integral_domain_without_division_tag&) const
  {
#ifdef AG2_PROFILE_PREDICATES
    ag2_predicate_profiler::side_of_bisector_counter++;
#endif

    // this function compares the distances of the point(x, y) from the
    // disks {(x1, y1), w1} and {(x2, y2), w2}
    RT D1 = CGAL::square(p1.x() - p.x()) + CGAL::square(p1.y() - p.y());
    RT D2 = CGAL::square(p2.x() - p.x()) + CGAL::square(p2.y() - p.y());
    RT Dw = p2.weight() - p1.weight();

    Sign sign_of_Dw = CGAL::sign(Dw);
    Comparison_result R = CGAL::compare(D1, D2);

    if ( sign_of_Dw == ZERO ) {
      return R;
    }
    if ( sign_of_Dw == POSITIVE ) {
      if ( R != SMALLER )  return LARGER;

      Sign s = sign_a_plus_b_x_sqrt_c<RT>(D1 - D2 + CGAL::square(Dw),
                                          RT(2) * Dw, D1);
      return ((s == POSITIVE) ? LARGER : ((s == ZERO) ? EQUAL : SMALLER));
    }

    if ( R != LARGER )  return SMALLER;
    Sign s = sign_a_plus_b_x_sqrt_c<RT>(D1 - D2 - CGAL::square(Dw),
                                        RT(2) * Dw, D2);

    return ((s == POSITIVE) ? LARGER : ((s == ZERO) ? EQUAL : SMALLER));
  }

  Comparison_result
  compare_distances(const Site_2& p1, const Site_2& p2,
                    const Point_2 &p, const Field_with_sqrt_tag&) const
  {
#ifdef AG2_PROFILE_PREDICATES
    ag2_predicate_profiler::side_of_bisector_counter++;
#endif
    // this function compares the distances of the point(x, y) from the
    // disks {(x1, y1), w1} and {(x2, y2), w2}

    RT D1 = CGAL::square(p1.x() - p.x()) + CGAL::square(p1.y() - p.y());
    RT D2 = CGAL::square(p2.x() - p.x()) + CGAL::square(p2.y() - p.y());

    RT d1 = CGAL::sqrt(D1) - p1.weight();
    RT d2 = CGAL::sqrt(D2) - p2.weight();

    return CGAL::compare(d1, d2);
  }

public:
  typedef Oriented_side                   result_type;
  struct argument_type {};

  inline
  Oriented_side operator()(const Site_2& p1, const Site_2& p2,
                           const Point_2 &p) const
  {
    Comparison_result r = compare_distances(p1, p2, p, Method_tag());

    if ( r == EQUAL ) { return ON_ORIENTED_BOUNDARY; }
    return ( r == LARGER ) ? ON_NEGATIVE_SIDE : ON_POSITIVE_SIDE;
  }

};


//--------------------------------------------------------------------

} //namespace ApolloniusGraph_2

} //namespace CGAL

#endif // CGAL_APOLLONIUS_GRAPH_2_ORIENTED_SIDE_OF_BISECTOR_C2_H
