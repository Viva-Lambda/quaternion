/*
MIT License

Copyright (c) 2021 Viva Lambda email <76657254+Viva-Lambda@users.noreply.github.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef QUATERNION_HPP
#define QUATERNION_HPP
#include <math.h>
#include <ostream>
#include <stdio.h>

namespace quat {
// holds quaternion related operations
typedef float real;

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
struct quat_c {
  QUATERNION_BASE base;
  real r;
  quat_c() {}
  quat_c(QUATERNION_BASE b, real a) : base(b), r(a) {}
  friend std::ostream &operator<<(std::ostream &out,
                                  const quat_c &c);
};

std::ostream &operator<<(std::ostream &out,
                         const quat_c &c) {
  switch (c.base) {
  case SCALAR_BASE: {
    out << "SCALAR_BASE::" << c.r << std::endl;
  }
  case I: {
    out << "I_BASE::" << c.r << std::endl;
  }
  case J: {
    out << "J_BASE::" << c.r << std::endl;
  }
  case K: {
    out << "K_BASE::" << c.r << std::endl;
  }
  }
  return out;
}

class quaternion {
public:
  quaternion() {
    coeffs[0] = 0;
    coeffs[1] = 1;
    coeffs[2] = 1;
    coeffs[3] = 1;
  }
  quaternion(real x, real y, real z, real w) {
    coeffs[0] = x;
    coeffs[1] = y;
    coeffs[2] = z;
    coeffs[3] = w;
  }
  quaternion(real c[4]) {
    for (unsigned int i = 0; i < 4; i++) {
      coeffs[i] = c[i];
    }
  }
  quaternion(real c1, const quat_c &qc2, const quat_c &qc3,
             const quat_c &qc4) {
    coeffs[0] = c1;
    coeffs[1] = qc2.r;
    coeffs[2] = qc3.r;
    coeffs[3] = qc4.r;
  }

  quaternion(const quat_c &c1, const quat_c &qc2,
             const quat_c &qc3, const quat_c &qc4) {
    coeffs[0] = c1.r;
    coeffs[1] = qc2.r;
    coeffs[2] = qc3.r;
    coeffs[3] = qc4.r;
  }
  quaternion(real c1, real cs[3]) {
    coeffs[0] = c1;
    coeffs[1] = cs[0];
    coeffs[2] = cs[1];
    coeffs[3] = cs[2];
  }
  real scalar() const { return coeffs[0]; }
  void vector(real v[3]) const {
    v[0] = x();
    v[1] = y();
    v[2] = z();
  }
  bool vector_multiplication(real t) {
    coeffs[1] *= t;
    coeffs[2] *= t;
    coeffs[3] *= t;
    return true;
  }
  bool vector_addition(real t) {
    coeffs[1] += t;
    coeffs[2] += t;
    coeffs[3] += t;
    return true;
  }
  bool vector_subtraction(real t) {
    coeffs[1] -= t;
    coeffs[2] -= t;
    coeffs[3] -= t;
    return true;
  }
  bool vector_division(real t) {
    if (t == 0)
      return false;
    coeffs[1] /= t;
    coeffs[2] /= t;
    coeffs[3] /= t;
    return true;
  }
  /** arithmetic operations with a scalar on vector part*/
  bool vector_multiplication(real v[3], real t) const {
    v[0] = x() * t;
    v[1] = y() * t;
    v[2] = z() * t;
    return true;
  }
  bool vector_addition(real v[3], real t) const {
    v[0] = x() + t;
    v[1] = y() + t;
    v[2] = z() + t;
    return true;
  }
  bool vector_subtraction(real v[3], real t) const {
    v[0] = x() - t;
    v[1] = y() - t;
    v[2] = z() - t;
    return true;
  }
  bool vector_division(real v[3], real t) const {
    if (t == 0)
      return false;
    v[0] = x() / t;
    v[1] = y() / t;
    v[2] = z() / t;
    return true;
  }
  /** arithmetic operations with a vector on vector part*/
  bool vector_multiplication(real v[3], real t[3]) const {
    v[0] = x() * t[0];
    v[1] = y() * t[1];
    v[2] = z() * t[2];
    return true;
  }
  bool vector_addition(real v[3], real t[3]) const {
    v[0] = x() + t[0];
    v[1] = y() + t[1];
    v[2] = z() + t[2];
    return true;
  }
  bool vector_subtraction(real v[3], real t[3]) const {
    v[0] = x() - t[0];
    v[1] = y() - t[1];
    v[2] = z() - t[2];
    return true;
  }
  bool vector_division(real v[3], real t[3]) const {
    for (unsigned int i = 0; i < 3; i++) {
      if (t[i] == 0)
        return false;
    }
    v[0] = x() / t[0];
    v[1] = y() / t[1];
    v[2] = z() / t[2];
    return true;
  }
  /** dot product and cross product for two vec3*/
  real vector_dot(real t[3]) const {
    return t[0] * x() + t[1] * y() + t[2] * z();
  }
  real vector_dot(real v[3], real t[3]) const {
    return t[0] * v[0] + t[1] * v[1] + t[2] * v[2];
  }
  bool vector_cross(real out[3], real t[3]) const {
    out[0] = y() * t[2] - z() * t[1];
    out[1] = z() * t[0] - x() * t[2];
    out[2] = x() * t[1] - y() * t[0];
    return true;
  }

  real r() const { return coeffs[0]; }
  real x() const { return coeffs[1]; }
  real y() const { return coeffs[2]; }
  real z() const { return coeffs[3]; }

  /** Quaternion product as it is shown by Vince 2011 p.
   * 63
   Given a quaternion \f[q_a = [s_a, a]\f] and
   another quaternion \f[q_b = [s_b, b]\f]
   Their product is equal to:
   \f[s_a s_b - a \cdot b, s_a b + s_b a + a \times b \f]
   */
  quaternion hamilton_product(const quaternion &q_b) const {
    // s_a, s_b, a, b
    real s_a = scalar();
    real s_b = q_b.scalar();
    real a[3];
    vector(a);
    real b[3];
    q_b.vector(b);

    // s_a * s_b
    real s_ab = s_a * s_b;

    // a \cdot b
    real a_dot_b = vector_dot(a, b);

    // a \times b
    real cross_ab[3];
    vector_cross(cross_ab, b);

    // s_a * b + s_b * a + a \times b
    real out[3];
    for (unsigned int i = 0; i < 3; i++) {
      out[i] = s_a * b[i] + s_b * a[i] + cross_ab[i];
    }
    return quaternion(s_ab - a_dot_b, out);
  }
  quaternion conjugate() const {
    real a1 = r();
    real b1 = x() * -1;
    real c1 = y() * -1;
    real d1 = z() * -1;
    return quaternion(a1, b1, c1, d1);
  }
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  quaternion normalized() const {
    real inv_mag = static_cast<real>(1.0) / norm();
    real scalar_part = scalar() * inv_mag;
    real vs[3];
    vector(vs);
    vector_multiplication(vs, inv_mag);
    return quaternion(scalar_part, vs);
  }
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  quaternion inversed() const {
    //
    real inv_mag2 = static_cast<real>(1.0) / det();
    quaternion conj = conjugate();
    real spart = conj.scalar() * inv_mag2;
    real vs[3];
    vector_multiplication(vs, inv_mag2);
    return quaternion(spart, vs);
  }
  /**
   \brief from Vince 2011 - Quaternions for Computer
   Graphics p. 69
   */
  quaternion operator+(const quaternion &q) const {
    real out[3];
    real vs[3];
    q.vector(vs);
    vector_addition(out, vs);
    return quaternion(scalar() + q.scalar(), out);
  }
  /**
   \brief from Vince 2011 - Quaternions for Computer
   Graphics p. 69
  */
  quaternion operator-(const quaternion &q) const {
    real out[3];
    real vs[3];
    q.vector(vs);
    vector_subtraction(out, vs);
    return quaternion(scalar() - q.scalar(), out);
  }
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  quaternion operator*(const quaternion &q) const {
    return hamilton_product(q);
  }
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  quaternion operator*(real r) const {
    real out[3];
    vector_multiplication(out, r);
    return quaternion(scalar() * r, out);
  }
  quaternion power(unsigned int i) const {
    quaternion accumulant = *this;
    quaternion result2 = *this;
    for (unsigned int j = 1; j < i; j++) {
      accumulant = accumulant.hamilton_product(result2);
    }
    return accumulant;
  }
  quaternion squared() const {
    quaternion r1 = *this;
    quaternion r2 = *this;
    return r1 * r2;
  }
  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 69
   */
  real norm() const { return sqrt(det()); }

  /**
    \brief from Vince 2011 - Quaternions for Computer
    Graphics p. 25
   */
  real determinant() const {
    real a2 = r() * r();
    real b2 = x() * x();
    real c2 = y() * y();
    real d2 = z() * z();
    return a2 + b2 + c2 + d2;
  }
  real det() const { return determinant(); }
  real magnitude() const { return norm(); }
  friend std::ostream &operator<<(std::ostream &out,
                                  const quaternion &q);

  bool get_component(std::size_t i, quat_c &c) const {
    if (i == 0) {
      quat_c c_;
      c_.r = coeffs[0];
      c = quat_c(SCALAR_BASE, coeffs[0]);
      return true;
    } else if (i == 1) {
      c = quat_c(I, coeffs[1]);
      return true;
    } else if (i == 2) {
      c = quat_c(J, coeffs[2]);
      return true;
    } else if (i == 3) {
      c = quat_c(K, coeffs[3]);
      return true;
    } else {
      return false;
    }
  }

private:
  real coeffs[4];
};
std::ostream &operator<<(std::ostream &out,
                         const quaternion &q) {
  out << q.r() << " + " << q.x() << "i"
      << " + " << q.y() << "j"
      << " + " << q.z() << "k" << std::endl;
  return out;
}
};

#endif
