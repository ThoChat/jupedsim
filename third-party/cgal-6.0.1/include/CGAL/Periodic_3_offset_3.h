// Copyright (c) 2006-2009   INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v6.0.1/Periodic_3_triangulation_3/include/CGAL/Periodic_3_offset_3.h $
// $Id: include/CGAL/Periodic_3_offset_3.h 50cfbde3b84 $
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Nico Kruithof <Nico.Kruithof@sophia.inria.fr>
//                 Manuel Caroli <Manuel.Caroli@sophia.inria.fr>

#ifndef CGAL_PERIODIC_3_OFFSET_H
#define CGAL_PERIODIC_3_OFFSET_H

#include <CGAL/license/Periodic_3_triangulation_3.h>

#include <CGAL/basic.h>
#include <CGAL/assertions.h>
#include <CGAL/Cartesian.h>

#include <iostream>
#include <type_traits>

namespace CGAL {

class Periodic_3_offset_3
{
  template <class K2>
  friend std::ostream & operator<<(std::ostream &os,
                                   const Periodic_3_offset_3 &off);

public:
  Periodic_3_offset_3() : _offx(0), _offy(0), _offz(0) {}
  Periodic_3_offset_3(int x, int y, int z) : _offx(x), _offy(y), _offz(z) {}

  bool is_null() const {
    return ((_offx | _offy | _offz) == 0);
  }

  int& x() { return _offx; }
  int x() const { return _offx; }
  int& y() { return _offy; }
  int y() const { return _offy; }
  int& z() { return _offz; }
  int z() const { return _offz; }

  // Use sfinae on the operator[] to accept only integral types as argument
  template <typename T,
            typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
  int& operator[](T i)
  {
    if (i==0) return _offx;
    if (i==1) return _offy;
    CGAL_assertion(i==2);
    return _offz;
  }

  template <typename T,
            typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
  int operator[](T i) const
  {
    if (i==0) return _offx;
    if (i==1) return _offy;
    CGAL_assertion(i==2);
    return _offz;
  }

  void operator+=(const Periodic_3_offset_3 &other) {
    _offx += other._offx;
    _offy += other._offy;
    _offz += other._offz;
  }
  void operator-=(const Periodic_3_offset_3 &other) {
    _offx -= other._offx;
    _offy -= other._offy;
    _offz -= other._offz;
  }
  Periodic_3_offset_3 operator-() const {
    return Periodic_3_offset_3(-_offx,-_offy,-_offz);
  }
  bool operator==(const Periodic_3_offset_3 &other) const {
    return ((_offx == other._offx) &&
            (_offy == other._offy) &&
            (_offz == other._offz));
  }
  bool operator!=(const Periodic_3_offset_3 &other) const {
    return  ((_offx != other._offx) ||
             (_offy != other._offy) ||
             (_offz != other._offz));
  }
  bool operator<(const Periodic_3_offset_3 &other) const {
    if (_offx != other._offx)
      return (_offx < other._offx);
    else {
      if (_offy != other._offy)
        return (_offy < other._offy);
      else {
        return (_offz < other._offz);
      }
    }
  }

  Periodic_3_offset_3 operator+(const Periodic_3_offset_3 &off2) const {
    return Periodic_3_offset_3(_offx+off2.x(), _offy+off2.y(), _offz+off2.z());
  }
  Periodic_3_offset_3 operator-(const Periodic_3_offset_3 &off2) const {
    return Periodic_3_offset_3(_offx-off2.x(), _offy-off2.y(), _offz-off2.z());
  }

private:
  int _offx, _offy, _offz;
};

template <class K>
inline Point_3<K> operator+(const Point_3<K> &p, const Periodic_3_offset_3 &off) {
  return (off.is_null() ? p : Point_3<K>(p.x()+off.x(), p.y()+off.y(), p.z()+off.z()));
}

inline std::ostream
&operator<<(std::ostream &os, const Periodic_3_offset_3 &off) {
  if (IO::is_ascii(os))
    os << off.x() << " " << off.y() << " " << off.z();
  else {
    write(os,off.x());
    write(os,off.y());
    write(os,off.z());
  }
  return os;
}

inline std::istream
&operator>>(std::istream &is, Periodic_3_offset_3 &off) {
  int x=0,y=0,z=0;
  if (IO::is_ascii(is))
    is >> x >> y >> z;
  else {
    read(is,x);
    read(is,y);
    read(is,z);
  }
  off = Periodic_3_offset_3(x,y,z);
  return is;
}

} //namespace CGAL

#endif // CGAL_PERIODIC_3_OFFSET_H
