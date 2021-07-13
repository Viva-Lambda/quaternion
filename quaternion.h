/*
MIT License

Copyright (c) 2021 Viva Lambda email
<76657254+Viva-Lambda@users.noreply.github.com>

Permission is hereby granted, free of charge, to any person
obtaining a copy
of this software and associated documentation files (the
"Software"), to deal
in the Software without restriction, including without
limitation the rights
to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall
be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef QUATERNION_H
#define QUATERNION_H

#include <cstdint>
#include <functional>
#include <math.h>
#include <ostream>
#include <stdio.h>

namespace quat11 {
// holds quaternion related operations
enum QUATERNION_FLAGS : std::uint_least8_t {
  SUCCESS = 1,
  SIZE_ERROR = 2,
  INDEX_ERROR = 3,
  ARG_ERROR = 4
};

/**Quaternion bases*/
enum QUATERNION_BASE {
  SCALAR_BASE, // null value for quaternion base it has no
               // effect on computation
  I,           // i base for the second quaternion component
  J,           // j base for the third quaternion component
  K            // k base for the fourth quaternion component
};

/**
  \brief Quaternion component
 */
template <class T> struct quat_c {
  QUATERNION_BASE base;
  T r;
  quat_c() {}
  quat_c(QUATERNION_BASE b, T a) : base(b), r(a) {}

  template <typename K>
  friend std::ostream &operator<<(std::ostream &out, const quat_c<T> &c);
};

template <class T> class quaternion {
public:
  quaternion()
      : coeffs{static_cast<T>(0), static_cast<T>(1), static_cast<T>(1),
               static_cast<T>(1)} {}

  quaternion(T x, T y, T z, T w) : coeffs{x, y, z, w} {}
  quaternion(const T c[4]) : coeffs{c[0], c[1], c[2], c[3]} {}

  //
  quaternion(T c1, const quat_c<T> &qc2, const quat_c<T> &qc3,
             const quat_c<T> &qc4)
      : coeffs{c1, qc2.r, qc3.r, qc4.r} {}

  quaternion(const quat_c<T> &c1, const quat_c<T> &qc2, const quat_c<T> &qc3,
             const quat_c<T> &qc4)
      : coeffs{c1.r, qc2.r, qc3.r, qc4.r} {}
  quaternion(T c1, T cs[3]) : coeffs{c1, cs[0], cs[1], cs[2]} {}

  QUATERNION_FLAGS scalar(T &out) const;
  QUATERNION_FLAGS vector(T v[3]) const;
  /** arithmetic operations with a scalar on vector part*/
  QUATERNION_FLAGS
  apply(T t, const std::function<T(T, T)> &fn, T out[3]) const;
  QUATERNION_FLAGS
  apply(T t[3], const std::function<T(T, T)> &fn, T out[3]) const;
  QUATERNION_FLAGS
  apply(const quaternion &q, const std::function<T(T, T)> &fn,
        quaternion<T> &out) const;
  QUATERNION_FLAGS
  apply(const T &q, const std::function<T(T, T)> &fn, quaternion<T> &out) const;

  QUATERNION_FLAGS vector_multiplication(T t, T out[3]) const;
  QUATERNION_FLAGS vector_addition(T t, T out[3]) const;
  QUATERNION_FLAGS vector_subtraction(T t, T out[3]) const;
  QUATERNION_FLAGS vector_division(T t, T out[3]) const;
  /** arithmetic operations with a vector on vector part*/
  QUATERNION_FLAGS vector_multiplication(T t[3], T out[3]) const;
  QUATERNION_FLAGS vector_addition(T t[3], T out[3]) const;
  QUATERNION_FLAGS vector_subtraction(T t[3], T out[3]) const;
  QUATERNION_FLAGS vector_division(T t[3], T out[3]) const;
  /** dot product and cross product for two vec3*/
  QUATERNION_FLAGS vector_dot(T t[3], T &out) const;
  QUATERNION_FLAGS vector_dot(T v[3], T t[3], T &out) const;
  QUATERNION_FLAGS vector_cross(T t[3], T out[3]) const;

  /** Quaternion product as it is shown by Vince 2011 p.
   * 63
   Given a quaternion \f[q_a = [s_a, a]\f] and
   another quaternion \f[q_b = [s_b, b]\f]
   Their product is equal to:
   \f[s_a s_b - a \cdot b, s_a b + s_b a + a \times b \f]
   */
  QUATERNION_FLAGS
  hamilton_product(const quaternion &q_b, quaternion<T> &out) const;
  QUATERNION_FLAGS conjugate(quaternion<T> &out) const;
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  QUATERNION_FLAGS normalized(quaternion<T> &out) const;
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  QUATERNION_FLAGS inversed(quaternion<T> &out) const;
  /**
   \brief from Vince 2011 - Quaternions for Computer
   Graphics p. 69
   */
  QUATERNION_FLAGS add(const quaternion &q, quaternion<T> &out) const;
  /**
   \brief from Vince 2011 - Quaternions for Computer
   Graphics p. 69
  */
  QUATERNION_FLAGS subtract(const quaternion &q, quaternion<T> &out) const;
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  QUATERNION_FLAGS product(const quaternion &q, quaternion<T> &out) const;
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  QUATERNION_FLAGS product(T r, quaternion<T> &out) const;
  QUATERNION_FLAGS power(unsigned int i, quaternion<T> &out) const;
  QUATERNION_FLAGS squared(quaternion<T> &out) const;
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  QUATERNION_FLAGS norm(T &out) const;

  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 25
   */
  QUATERNION_FLAGS determinant(T &out) const;
  QUATERNION_FLAGS det(T &out) const;
  QUATERNION_FLAGS magnitude(T &out) const;
  template <typename K>
  friend std::ostream &operator<<(std::ostream &out, const quaternion<T> &q);

  QUATERNION_FLAGS get_component(std::size_t i, quat_c<T> &c) const;

private:
  T coeffs[4];
};
} // namespace quat11

#endif
