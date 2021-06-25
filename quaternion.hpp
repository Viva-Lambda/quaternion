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
  N, // null value for quaternion base it has no effect on
     // computation
  I, // i base for the second quaternion component
  J, // j base for the third quaternion component
  K  // k base for the fourth quaternion component
};

/**
  \brief Quaternion component
 */
struct quat_c {
  QUATERNION_BASE base;
  real r;
  quat_c() {}
  quat_c(QUATERNION_BASE b, real a) : base(b), r(a) {}
};

class quaternion {
public:
  quaternion() {}
  quaternion(real x, real y, real z, real w)
      : coeffs{x, y, z, w} {}
  quaternion(real c[4]) : coeffs{c[0], c[1], c[2], c[3]} {}
  quaternion(real c1, const quat_c &qc2, const quat_c &qc3,
             const quat_c &qc4)
      : coeffs{c1, qc2.r, qc3.r, qc4.r} {}
  quaternion(real c1, real cs[3])
      : coeffs{c1, cs[0], cs[1], cs[2]} {}
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

  real r() const { return coeffs[0]; }
  real x() const { return coeffs[1]; }
  real y() const { return coeffs[2]; }
  real z() const { return coeffs[3]; }

  quaternion hamilton_product(const quaternion &q) const {
    //
    real a1 = r();
    real a2 = q.r();
    real b1 = x();
    real b2 = q.x();
    real c1 = y();
    real c2 = q.y();
    real d1 = z();
    real d2 = q.z();
    real a12 = a1 * a2;
    real b12 = b1 * b2;
    real c12 = c1 * c2;
    real d12 = d1 * d2;
    real r_val = a12 - b12 - c12 - d12;
    real b_val = a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2;
    real c_val = a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2;
    real d_val = a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2;
    return quaternion(r_val, b_val, c_val, d_val);
  }
  quaternion conjugate() const {
    real a1 = r();
    real b1 = x();
    real c1 = y();
    real d1 = z();
    return quaternion(a1, -b1, -c1, -d1);
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
    quaternion result = *this;
    for (unsigned int j = 0; j < i; j++) {
      result = *this * result;
    }
    return result;
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
      c = quat_c(N, coeffs[0]);
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
  real coeffs[4] = {0, 1, 1, 1};
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
