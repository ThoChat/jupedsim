// Copyright (c) 2014
// INRIA Saclay-Ile de France (France)
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/NewKernel_d/include/CGAL/argument_swaps.h $
// $Id: include/CGAL/argument_swaps.h 50cfbde3b84 $
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Marc Glisse

#ifndef CGAL_ARGUMENT_SWAPS_H
#define CGAL_ARGUMENT_SWAPS_H

#include <CGAL/config.h>
#include <utility>

namespace CGAL {
namespace internal {

template<std::size_t,class...> struct Apply_to_last_then_rest_;

template<std::size_t d,class F,class T,class... U>
struct Apply_to_last_then_rest_<d,F,T,U...> {
        typedef typename Apply_to_last_then_rest_<d-1,F,U...,T>::result_type result_type;
        inline result_type operator()(F&&f,T&&t,U&&...u)const{
                return Apply_to_last_then_rest_<d-1,F,U...,T>()(
                                std::forward<F>(f),
                                std::forward<U>(u)...,
                                std::forward<T>(t));
        }
};

template<class F,class T,class... U>
struct Apply_to_last_then_rest_<0,F,T,U...> {
        typedef decltype(std::declval<F>()(std::declval<T>(), std::declval<U>()...)) result_type;
        inline result_type operator()(F&&f,T&&t,U&&...u)const{
        return std::forward<F>(f)(std::forward<T>(t), std::forward<U>(u)...);
        }
};
} // namespace internal

struct Apply_to_last_then_rest {
        template<class F,class T,class...U> inline
        typename internal::Apply_to_last_then_rest_<sizeof...(U),F,T,U...>::result_type
        operator()(F&&f,T&&t,U&&...u)const{
        return internal::Apply_to_last_then_rest_<sizeof...(U),F,T,U...>()(
                        std::forward<F>(f),
                        std::forward<T>(t),
                        std::forward<U>(u)...);
        }
};

} // namespace CGAL

#endif // CGAL_ARGUMENT_SWAPS_H
