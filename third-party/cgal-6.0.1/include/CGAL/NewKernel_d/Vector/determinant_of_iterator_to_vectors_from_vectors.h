// Copyright (c) 2014
// INRIA Saclay-Ile de France (France)
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/NewKernel_d/include/CGAL/NewKernel_d/Vector/determinant_of_iterator_to_vectors_from_vectors.h $
// $Id: include/CGAL/NewKernel_d/Vector/determinant_of_iterator_to_vectors_from_vectors.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Marc Glisse

#ifndef CGAL_VECTOR_DET_ITER_VEC_VEC_H
#define CGAL_VECTOR_DET_ITER_VEC_VEC_H
#include <CGAL/NewKernel_d/functor_tags.h>
#include <CGAL/Dimension.h>
#include <CGAL/use.h>

namespace CGAL {

template <class LA, class Dim_=typename LA::Dimension,
         class Max_dim_=typename LA::Max_dimension,
         bool = LA::template Property<Has_determinant_of_iterator_to_vectors_tag>::value,
         bool = LA::template Property<Has_determinant_of_vectors_tag>::value>
struct Add_determinant_of_iterator_to_vectors_from_vectors : LA {
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
};

//FIXME: Use variadics and boost so it works in any dimension.
template <class LA, class Max_dim_>
struct Add_determinant_of_iterator_to_vectors_from_vectors
<LA, Dimension_tag<2>, Max_dim_, false, true> : LA {
  typedef typename LA::NT NT;
  typedef typename LA::Vector Vector;
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
  template<class P,class=void> struct Property : LA::template Property<P> {};
  template<class D> struct Property<Has_determinant_of_iterator_to_vectors_tag, D> :
    std::true_type {};

  template<class Iter>
  static NT determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::determinant_of_vectors(a,b);
  }
  template<class Iter>
  static Sign sign_of_determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::sign_of_determinant_of_vectors(a,b);
  }
};

template <class LA, class Max_dim_>
struct Add_determinant_of_iterator_to_vectors_from_vectors
<LA, Dimension_tag<3>, Max_dim_, false, true> : LA {
  typedef typename LA::NT NT;
  typedef typename LA::Vector Vector;
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
  template<class P,class=void> struct Property : LA::template Property<P> {};
  template<class D> struct Property<Has_determinant_of_iterator_to_vectors_tag, D> :
    std::true_type {};

  template<class Iter>
  static NT determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::determinant_of_vectors(a,b,c);
  }
  template<class Iter>
  static Sign sign_of_determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::sign_of_determinant_of_vectors(a,b,c);
  }
};

template <class LA, class Max_dim_>
struct Add_determinant_of_iterator_to_vectors_from_vectors
<LA, Dimension_tag<4>, Max_dim_, false, true> : LA {
  typedef typename LA::NT NT;
  typedef typename LA::Vector Vector;
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
  template<class P,class=void> struct Property : LA::template Property<P> {};
  template<class D> struct Property<Has_determinant_of_iterator_to_vectors_tag, D> :
    std::true_type {};

  template<class Iter>
  static NT determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::determinant_of_vectors(a,b,c,d);
  }
  template<class Iter>
  static Sign sign_of_determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::sign_of_determinant_of_vectors(a,b,c,d);
  }
};

template <class LA, class Max_dim_>
struct Add_determinant_of_iterator_to_vectors_from_vectors
<LA, Dimension_tag<5>, Max_dim_, false, true> : LA {
  typedef typename LA::NT NT;
  typedef typename LA::Vector Vector;
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
  template<class P,class=void> struct Property : LA::template Property<P> {};
  template<class D> struct Property<Has_determinant_of_iterator_to_vectors_tag, D> :
    std::true_type {};

  template<class Iter>
  static NT determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; ++first;
    Vector const&e=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::determinant_of_vectors(a,b,c,d,e);
  }
  template<class Iter>
  static Sign sign_of_determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; ++first;
    Vector const&e=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::sign_of_determinant_of_vectors(a,b,c,d,e);
  }
};

template <class LA, class Max_dim_>
struct Add_determinant_of_iterator_to_vectors_from_vectors
<LA, Dimension_tag<6>, Max_dim_, false, true> : LA {
  typedef typename LA::NT NT;
  typedef typename LA::Vector Vector;
  template< class D2, class D3=D2 >
    struct Rebind_dimension {
      typedef typename LA::template Rebind_dimension<D2,D3> LA2;
      typedef Add_determinant_of_iterator_to_vectors_from_vectors<LA2> Other;
    };
  template<class P,class=void> struct Property : LA::template Property<P> {};
  template<class D> struct Property<Has_determinant_of_iterator_to_vectors_tag, D> :
    std::true_type {};

  template<class Iter>
  static NT determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; ++first;
    Vector const&e=*first; ++first;
    Vector const&f=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::determinant_of_vectors(a,b,c,d,e,f);
  }
  template<class Iter>
  static Sign sign_of_determinant_of_iterator_to_vectors(Iter const&first, Iter const&end){
    Vector const&a=*first; ++first;
    Vector const&b=*first; ++first;
    Vector const&c=*first; ++first;
    Vector const&d=*first; ++first;
    Vector const&e=*first; ++first;
    Vector const&f=*first; CGAL_assertion(++first==end);
    CGAL_USE(end);
    return LA::sign_of_determinant_of_vectors(a,b,c,d,e,f);
  }
};

}
#endif
