// Copyright (c) 2005-2006  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Surface_mesher/include/CGAL/Surface_mesher/Vertices_on_the_same_surface_criterion.h $
// $Id: include/CGAL/Surface_mesher/Vertices_on_the_same_surface_criterion.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent RINEAU

#ifndef CGAL_VERTICES_ON_THE_SAME_SURFACE_CRITERION_H
#define CGAL_VERTICES_ON_THE_SAME_SURFACE_CRITERION_H

#include <CGAL/license/Surface_mesher.h>

#define CGAL_DEPRECATED_HEADER "<CGAL/Surface_mesher/Vertices_on_the_same_surface_criterion.h>"
#define CGAL_DEPRECATED_MESSAGE_DETAILS \
  "The 3D Mesh Generation package (see https://doc.cgal.org/latest/Mesh_3/) should be used instead."
#include <CGAL/Installation/internal/deprecation_warning.h>

#include <CGAL/Surface_mesher/Standard_criteria.h>

namespace CGAL {

  namespace Surface_mesher {

template <typename Tr>
class Vertices_on_the_same_surface_criterion :
    public Refine_criterion <Tr> {
  public:
    typedef Refine_criterion <Tr> Criterion;
    typedef typename Criterion::Quality Quality;

  private:
    typedef typename Tr::Facet Facet;
    typedef typename Tr::Vertex_handle Vertex_handle;
    typedef typename Tr::Cell_handle Cell_handle;

  public:
  bool is_bad (const Facet& f, Quality& q) const {
      const Cell_handle& ch = f.first;
      const int i = f.second;
      const Vertex_handle& v1 = ch->vertex((i+1)&3);
      const Vertex_handle& v2 = ch->vertex((i+2)&3);
      const Vertex_handle& v3 = ch->vertex((i+3)&3);

      const int& number = v1->point().surface_index();
      if ( number == 0 ||
           (v2->point().surface_index() != number) ||
           (v3->point().surface_index() != number ) )
      {
        q = Quality(0);
        return true;
      }
      else
      {
        q = Quality(1);
        return false;
      }
    }
}; // end Vertices_on_the_same_surface_criterion


} // end namespace Surface_mesher

} // end namespace CGAL


#endif // CGAL_VERTICES_ON_THE_SAME_SURFACE_CRITERION_H
