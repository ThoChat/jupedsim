// Copyright (c) 2005,2006,2007,2009,2010,2011 Tel-Aviv University (Israel).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Arrangement_on_surface_2/include/CGAL/Arr_point_location/Td_inactive_vertex.h $
// $Id: include/CGAL/Arr_point_location/Td_inactive_vertex.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)         : Oren Nechushtan <theoren@math.tau.ac.il>
//               updated by: Michal Balas <balasmic@post.tau.ac.il>

#ifndef CGAL_TD_INACTIVE_VERTEX_H
#define CGAL_TD_INACTIVE_VERTEX_H

#include <CGAL/license/Arrangement_on_surface_2.h>


/*! \file
 * Definition of the Td_inactive_vertex<Td_traits> class.
 */

#include <CGAL/Arr_point_location/Trapezoidal_decomposition_2.h>
#include <variant>


#ifdef CGAL_TD_DEBUG
#define CGAL_TD_INLINE
#else
#define CGAL_TD_INLINE inline
#endif

namespace CGAL {

/*! \class
 * Implementation of a pseudo-trapezoid as two halfedges(top,bottom)
 * and two curve-ends(left,right).
 * Trapezoids are represented as two curve-ends called right and left and
 * two halfedges called top and bottom. The curve-ends (points) lie on the
 * right and left boundaries of the trapezoid respectively and the halfedges
 * bound the trapezoid from above and below.
 * There exist degenerate trapezoids called infinite trapezoid; this happens
 * when one of the four sides is on the parameter space boundary.
 * Trapezoids are created as active and become inactive when Remove() member
 * function called.
 * Each trapezoid has at most four neighboring trapezoids.
 * X_trapezoid structure can represent a real trapezoid, a Td-edge or an
 * edge-end (end point).
 */
template <class Td_traits_>
class Td_inactive_vertex : public Handle
{
public:

  //type of traits class
  typedef Td_traits_                                   Traits;

  //type of point (Point_2)
  typedef typename Traits::Point                       Point;

  //type of X_monotone_curve_2
  typedef typename Traits::X_monotone_curve_2     X_monotone_curve_2;

  //type of Curve_end
  typedef typename Traits::Curve_end              Curve_end;

  //type of Halfedge_const_handle (trapezoid edge)
  typedef typename Traits::Halfedge_const_handle  Halfedge_const_handle;

  //type of Vertex_const_handle (trapezoid vertex)
  typedef typename Traits::Vertex_const_handle    Vertex_const_handle;

  //type of Td_inactive_vertex (Self)
  typedef typename Traits::Td_inactive_vertex            Self;

  typedef typename Traits::Td_map_item            Td_map_item;

  //type of Trapezoidal decomposition
  typedef Trapezoidal_decomposition_2<Traits>          TD;

  //type of In face iterator
  typedef typename TD::In_face_iterator                In_face_iterator;

  //type of Trapezoidal map search structure
  typedef typename TD::Dag_node                 Dag_node;


  //friend class declarations:

  friend class Trapezoidal_decomposition_2<Traits>;

#ifdef CGAL_PM_FRIEND_CLASS
#if defined(__SUNPRO_CC) || defined(__PGI) || defined(__INTEL_COMPILER)
  friend class Trapezoidal_decomposition_2<Traits>::In_face_iterator;
#elif (__GNUC__ > 0)
  friend class Trapezoidal_decomposition_2<Traits>::In_face_iterator;
#else
  friend class In_face_iterator;
#endif
#endif

   /*! \class
   * Inner class Data derived from Rep class
   */
  class Data : public Rep
  {
    friend class Td_inactive_vertex<Td_traits_>;

  public:
    //c'tors
    Data (Point _p, Dag_node* _p_node): p(_p),p_node(_p_node)
    { }

    ~Data() { }

  protected:
    Point p;
    Dag_node* p_node;
  };


 private:

  Data* ptr() const { return (Data*)(PTR);  }


#ifndef CGAL_TD_DEBUG
#ifdef CGAL_PM_FRIEND_CLASS
 protected:
#else
 public: // workaround
#endif
#else //CGAL_TD_DEBUG
 public:
#endif //CGAL_TD_DEBUG


  /*! Set the DAG node. */
  inline void set_dag_node(Dag_node* p)
  {
    ptr()->p_node = p;
  }


  inline void set_point(Vertex_const_handle v_before_rem)
  {
    ptr()->p = v_before_rem->point();
  }


 public:

  /// \name Constructors.
  //@{

  /*! Constructor given Vertex & Halfedge handles. */
  Td_inactive_vertex (Vertex_const_handle v_before_rem, Dag_node* node = nullptr)
  {
    PTR = new Data(v_before_rem->point(), node);
  }


  /*! Copy constructor. */
  Td_inactive_vertex (const Self& tr) : Handle(tr)
  {
  }

  //@}

  /// \name Operator overloading.
  //@{

  /*! Assignment operator.
  *   operator= should not copy m_dag_node (or otherwise update
  *     Dag_node::replace)
    */
  inline Self& operator= (const Self& t2)
  {
          Handle::operator=(t2);
          return *this;
  }

  /*! Operator==. */
  inline bool operator== (const Self& t2) const
  {
    return (ptr() == t2.ptr());
  }

  /*! Operator!=. */
  inline bool operator!= (const Self& t2) const
  {
    return !(operator==(t2));
  }

  //@}


  /// \name Access methods.
  //@{

  inline Self& self()
  {
    return *this;
  }

  inline const Self& self() const
  {
    return *this;
  }

  /*! Access the trapezoid id (PTR). */
  inline unsigned long id() const
  {
    return (unsigned long) PTR;
  }

  inline Point& point() const
  {
    return ptr()->p;
  }

  /*! Access DAG node. */
  Dag_node* dag_node() const
  {
    return ptr()->p_node;
  }


  //@}



};

} //namespace CGAL

#endif
