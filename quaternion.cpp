#include "quaternion.h"

using namespace quat11;

namespace quat11 {

template <typename T>
std::ostream &operator<<(std::ostream &out, const quat_c<T> &c) {
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

template <class T> QUATERNION_FLAGS quaternion<T>::scalar(T &out) const {
  out = coeffs[0];
  return SUCCESS;
}
template <class T> QUATERNION_FLAGS quaternion<T>::vector(T v[3]) const {
  v[0] = coeffs[1];
  v[1] = coeffs[2];
  v[2] = coeffs[3];
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::apply(T t, const std::function<T(T, T)> &fn,
                                      T out[3]) const {
  T vec[3];
  auto res = vector(vec);

  if (res != SUCCESS)
    return res;

  out[0] = fn(vec[0], t);
  out[1] = fn(vec[1], t);
  out[2] = fn(vec[2], t);
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::apply(T t[3], const std::function<T(T, T)> &fn,
                                      T out[3]) const {
  T vec[3];
  auto res = vector(vec);
  if (res != SUCCESS)
    return res;
  out[0] = fn(vec[0], t[0]);
  out[1] = fn(vec[1], t[1]);
  out[2] = fn(vec[2], t[2]);
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::apply(const quaternion &q,
                                      const std::function<T(T, T)> &fn,
                                      quaternion<T> &out) const {
  //
  T s = static_cast<T>(0);
  T qs = static_cast<T>(0);
  auto res = scalar(s);
  if (res != SUCCESS)
    return res;
  res = q.scalar(qs);
  if (res != SUCCESS)
    return res;
  //
  T sresult = fn(s, qs);

  T ovec[3];
  T qvec[3];

  res = q.vector(qvec);
  if (res != SUCCESS)
    return res;

  //
  apply(qvec, fn, ovec);

  // o
  out = quaternion(sresult, ovec);
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::apply(const T &q,
                                      const std::function<T(T, T)> &fn,
                                      quaternion<T> &out) const {
  T s = static_cast<T>(0);
  auto res = scalar(s);
  if (res != SUCCESS)
    return res;

  // scalar result
  T sres = fn(s, q);

  T vec[3];
  res = vector(vec);
  if (res != SUCCESS)
    return res;

  //
  T ovec[3];
  ovec[0] = fn(vec[0], q);
  ovec[1] = fn(vec[1], q);
  ovec[2] = fn(vec[2], q);
  //
  out = quaternion(sres, ovec);
  return SUCCESS;
}

template <class T>
QUATERNION_FLAGS quaternion<T>::vector_multiplication(T t, T out[3]) const {
  auto fn = [](T thisval, T tval) { return thisval * tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_addition(T t, T out[3]) const {
  auto fn = [](T thisval, T tval) { return thisval + tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_subtraction(T t, T out[3]) const {
  auto fn = [](T thisval, T tval) { return thisval - tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_division(T t, T out[3]) const {
  if (t == 0)
    return ARG_ERROR;
  auto fn = [](T thisval, T tval) { return thisval / tval; };
  return apply(t, fn, out);
}
/** arithmetic operations with a vector on vector part*/
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_multiplication(T t[3], T out[3]) const {
  auto fn = [](T thisval, T tval) { return thisval * tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_addition(T t[3], T out[3]) const {

  auto fn = [](T thisval, T tval) { return thisval + tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_subtraction(T t[3], T out[3]) const {

  auto fn = [](T thisval, T tval) { return thisval - tval; };
  return apply(t, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_division(T t[3], T out[3]) const {
  for (unsigned int i = 0; i < 3; i++) {
    if (t[i] == 0)
      return ARG_ERROR;
  }
  auto fn = [](T thisval, T tval) { return thisval / tval; };
  return apply(t, fn, out);
}
/** dot product and cross product for two vec3*/
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_dot(T t[3], T &out) const {
  T v[3];
  auto res = vector(v);
  if (res != SUCCESS)
    return res;
  return vector_dot(v, t, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_dot(T v[3], T t[3], T &out) const {
  out = t[0] * v[0] + t[1] * v[1] + t[2] * v[2];
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::vector_cross(T t[3], T out[3]) const {
  T vec[3];
  auto res = vector(vec);
  if (res != SUCCESS)
    return res;
  //
  out[0] = vec[1] * t[2] - vec[2] * t[1];
  out[1] = vec[2] * t[0] - vec[0] * t[2];
  out[2] = vec[0] * t[1] - vec[1] * t[0];
  return SUCCESS;
}

/** Quaternion product as it is shown by Vince 2011 p.
 * 63
 Given a quaternion \f[q_a = [s_a, a]\f] and
 another quaternion \f[q_b = [s_b, b]\f]
 Their product is equal to:
 \f[s_a s_b - a \cdot b, s_a b + s_b a + a \times b \f]
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::hamilton_product(const quaternion &q_b,
                                                 quaternion<T> &out) const {
  // s_a, s_b, a, b
  T s_a = static_cast<T>(0);
  auto res = scalar(s_a);
  if (res != SUCCESS)
    return res;

  T s_b = static_cast<T>(0);
  res = q_b.scalar(s_b);
  if (res != SUCCESS)
    return res;

  T a[3];
  res = vector(a);
  if (res != SUCCESS)
    return res;

  T b[3];
  res = q_b.vector(b);
  if (res != SUCCESS)
    return res;

  // s_a * s_b
  T s_ab = s_a * s_b;

  // a \cdot b
  T a_dot_b = static_cast<T>(0);
  res = vector_dot(a, b, a_dot_b);
  if (res != SUCCESS)
    return res;

  // a \times b
  T cross_ab[3];
  res = vector_cross(b, cross_ab);
  if (res != SUCCESS)
    return res;

  // s_a * b + s_b * a + a \times b
  T tout[3];
  for (unsigned int i = 0; i < 3; i++) {
    tout[i] = s_a * b[i] + s_b * a[i] + cross_ab[i];
  }
  out = quaternion(s_ab - a_dot_b, tout);
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::conjugate(quaternion<T> &out) const {
  T s = static_cast<T>(0);
  auto res = scalar(s);
  if (res != SUCCESS)
    return res;

  T vec[3];
  res = vector(vec);

  if (res != SUCCESS)
    return res;

  res = vector_multiplication(static_cast<T>(-1), vec);
  if (res != SUCCESS)
    return res;

  out = quaternion(s, vec);
  return SUCCESS;
}
/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 69
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::normalized(quaternion<T> &out) const {
  T nval = static_cast<T>(0);
  auto res = norm(nval);
  if (res != SUCCESS)
    return res;
  T inv_mag = static_cast<T>(1.0) / nval;

  res = scalar(nval);

  if (res != SUCCESS)
    return res;

  T scalar_part = nval * inv_mag;
  T vs[3];
  res = vector(vs);
  if (res != SUCCESS)
    return res;
  res = vector_multiplication(inv_mag, vs);

  if (res != SUCCESS)
    return res;
  out = quaternion(scalar_part, vs);
  return SUCCESS;
}
/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 69
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::inversed(quaternion<T> &out) const {
  //
  T det_out = static_cast<T>(0);
  auto res = det(det_out);
  if (res != SUCCESS)
    return res;

  T inv_mag2 = static_cast<T>(1.0) / det_out;
  quaternion conj;

  res = conjugate(conj);
  if (res != SUCCESS)
    return res;

  T conj_scalar = static_cast<T>(0);
  res = conj.scalar(conj_scalar);
  if (res != SUCCESS)
    return res;

  T spart = conj_scalar * inv_mag2;

  T vs[3];
  res = vector_multiplication(inv_mag2, vs);
  if (res != SUCCESS)
    return res;

  out = quaternion(spart, vs);
  return SUCCESS;
}
/**
 \brief from Vince 2011 - Quaternions for Computer
 Graphics p. 69
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::add(const quaternion &q,
                                    quaternion<T> &out) const {
  auto fn = [](T thisval, T tval) { return thisval + tval; };
  return apply(q, fn, out);
}
/**
 \brief from Vince 2011 - Quaternions for Computer
 Graphics p. 69
*/
template <class T>
QUATERNION_FLAGS quaternion<T>::subtract(const quaternion &q,
                                         quaternion<T> &out) const {
  auto fn = [](T thisval, T tval) { return thisval - tval; };
  return apply(q, fn, out);
}
/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 69
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::product(const quaternion &q,
                                        quaternion<T> &out) const {
  return hamilton_product(q, out);
}
/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 69
 */
template <class T>
QUATERNION_FLAGS quaternion<T>::product(T r, quaternion<T> &out) const {
  auto fn = [](T thisval, T tval) { return thisval * tval; };
  return apply(r, fn, out);
}
template <class T>
QUATERNION_FLAGS quaternion<T>::power(unsigned int i,
                                      quaternion<T> &out) const {
  quaternion accumulant = *this;
  quaternion result2 = *this;
  for (unsigned int j = 1; j < i; j++) {
    accumulant.hamilton_product(result2, accumulant);
  }
  out = accumulant;
  return SUCCESS;
}
template <class T>
QUATERNION_FLAGS quaternion<T>::squared(quaternion<T> &out) const {
  quaternion r1 = *this;
  quaternion r2 = *this;
  auto res = product(r1, out);
  if (res != SUCCESS)
    return res;

  res = product(r2, out);
  if (res != SUCCESS)
    return res;
  return SUCCESS;
}
/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 69
 */
template <class T> QUATERNION_FLAGS quaternion<T>::norm(T &out) const {
  auto res = det(out);

  if (res != SUCCESS)
    return res;
  //
  out = sqrt(out);
  return SUCCESS;
}

/**
  \brief from Vince 2011 - Quaternions for Computer
  Graphics p. 25
 */
template <class T> QUATERNION_FLAGS quaternion<T>::determinant(T &out) const {
  T s = static_cast<T>(0);
  auto res = scalar(s);
  if (res != SUCCESS)
    return res;
  T vec[3];
  res = vector(vec);
  if (res != SUCCESS)
    return res;

  T a2 = s * s;
  T b2 = vec[0] * vec[0];
  T c2 = vec[1] * vec[1];
  T d2 = vec[2] * vec[2];
  out = a2 + b2 + c2 + d2;
  return SUCCESS;
}
template <class T> QUATERNION_FLAGS quaternion<T>::det(T &out) const {
  return determinant(out);
}
template <class T> QUATERNION_FLAGS quaternion<T>::magnitude(T &out) const {
  return norm(out);
}

template <class T>
QUATERNION_FLAGS quaternion<T>::get_component(std::size_t i,
                                              quat_c<T> &c) const {
  if (i == 0) {
    quat_c<T> c_;
    c_.r = coeffs[0];
    c = quat_c<T>(SCALAR_BASE, coeffs[0]);
    return SUCCESS;
  } else if (i == 1) {
    c = quat_c<T>(I, coeffs[1]);
    return SUCCESS;
  } else if (i == 2) {
    c = quat_c<T>(J, coeffs[2]);
    return SUCCESS;
  } else if (i == 3) {
    c = quat_c<T>(K, coeffs[3]);
    return SUCCESS;
  } else {
    return ARG_ERROR;
  }
}
template <typename T>
std::ostream &operator<<(std::ostream &out, const quaternion<T> &q) {
  out << q.r() << " + " << q.x() << "i"
      << " + " << q.y() << "j"
      << " + " << q.z() << "k" << std::endl;
  return out;
}

#define CHECK(call)                                                            \
  do {                                                                         \
    QUATERNION_FLAGS res = call;                                               \
    return res == SUCCESS;                                                     \
  } while (0)

#define INFO(call)                                                             \
  do {                                                                         \
    QUATERNION_FLAGS res = call;                                               \
    switch (res) {                                                             \
    case SUCCESS: {                                                            \
      break;                                                                   \
    }                                                                          \
    case SIZE_ERROR: {                                                         \
      std::cout << "SIZE_ERROR "                                               \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    case INDEX_ERROR: {                                                        \
      std::cout << "INDEX_ERROR at "                                           \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    case INCORRECT_ARG_ERROR: {                                                \
      std::cout << "ARG_ERROR at "                                             \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    }                                                                          \
    return res;                                                                \
  } while (0)

#define INFO_VERBOSE(call)                                                     \
  do {                                                                         \
    QUATERNION_FLAGS res = call;                                                      \
    switch (res) {                                                             \
    case SUCCESS: {                                                            \
      std::cout << "SUCCESS" << std::endl;                                     \
      break;                                                                   \
    }                                                                          \
    case SIZE_ERROR: {                                                         \
      std::cout << "SIZE_ERROR "                                               \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    case INDEX_ERROR: {                                                        \
      std::cout << "INDEX_ERROR "                                              \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    case INCORRECT_ARG_ERROR: {                                                \
      std::cout << "ARG_ERROR "                                                \
                << " :: " << __FILE__ << " :: " << __LINE__ << std::endl;      \
    }                                                                          \
    }                                                                          \
    return res;                                                                \
  } while (0)

} // namespace quat11
