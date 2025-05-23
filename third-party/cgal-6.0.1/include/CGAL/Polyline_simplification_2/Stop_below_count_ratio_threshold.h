// Copyright (c) 2012 Geometry Factory. All rights reserved.
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Polyline_simplification_2/include/CGAL/Polyline_simplification_2/Stop_below_count_ratio_threshold.h $
// $Id: include/CGAL/Polyline_simplification_2/Stop_below_count_ratio_threshold.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Andreas Fabri, Fernando Cacciola
//
#ifndef CGAL_POLYLINE_SIMPLIFICATION_2_STOP_BELOW_COUNT_RATIO_THRESHOLD_H
#define CGAL_POLYLINE_SIMPLIFICATION_2_STOP_BELOW_COUNT_RATIO_THRESHOLD_H

#include <CGAL/license/Polyline_simplification_2.h>


#include <CGAL/Constrained_triangulation_plus_2.h>

namespace CGAL {

namespace Polyline_simplification_2
{

/// \ingroup PkgPolylineSimplification2Classes

/// This class is a stop predicate returning `true` when the percentage
/// of remaining vertices is smaller than a certain threshold.
///
/// \cgalModels{PolylineSimplificationStopPredicate}
class Stop_below_count_ratio_threshold
{
public :

  /// Initializes it with the given threshold value.
  Stop_below_count_ratio_threshold( double threshold ) : mThres(threshold) {}

  /// Returns `true` when `( current_count / initial_count )` is smaller or equal than the threshold.
  /// \tparam CDT  must be `CGAL::Constrained_Delaunay_triangulation_2` with a vertex type that
  /// is model of  `PolylineSimplificationVertexBase_2`.

  template<class CDT>
  bool operator()(const Constrained_triangulation_plus_2<CDT> &
                  , typename Constrained_triangulation_plus_2<CDT>::Vertex_handle
                  , typename CDT::Geom_traits::FT
                 , std::size_t                             initial_count
                 , std::size_t                             current_count
                 ) const
  {
    return ( (double)current_count / (double)initial_count ) <= mThres ;
  }

private:

  double mThres ;
};


} // namespace Polyline_simplification_2

} //namespace CGAL

#endif // CGAL_POLYLINE_SIMPLIFICATION_2_STOP_BELOW_COUNT_RATIO_THRESHOLD_H

