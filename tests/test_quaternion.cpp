// test file for quaternion
#include "../quaternion.hpp"
#include <ctest.h>

using namespace quat;

/*! @{
  Test accessors of quaternion object. These functions help
  us to access
  real parts, that is the scalar and the vector part of the
  quaternion object.
 */

typedef float real;

CTEST(suite, test_scalar1) {
  quaternion<real> q;
  ASSERT_EQUAL(q.scalar(), 0);
}
CTEST(suite, test_vector1) {
  quaternion<real> q;
  real vs[3];
  q.vector(vs);
  real comp[3] = {1, 1, 1};
  ASSERT_EQUAL(vs[0], comp[0]);
  ASSERT_EQUAL(vs[1], comp[1]);
  ASSERT_EQUAL(vs[2], comp[2]);
};
CTEST(suite, test_r) {
  quaternion<real> q;
  ASSERT_EQUAL(q.r(), 0);
}

CTEST(suite, test_x) {
  quaternion<real> q;
  ASSERT_EQUAL(q.x(), 1);
}
CTEST(suite, test_y) {
  quaternion<real> q;
  ASSERT_EQUAL(q.y(), 1);
}
CTEST(suite, test_z) {
  quaternion<real> q;
  ASSERT_EQUAL(q.z(), 1);
}

/*! @{ test component accessors */
CTEST(suite, test_get_component_0) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  bool result = q.get_component(0, comp);
  ASSERT_EQUAL(result, true);
  ASSERT_EQUAL(comp.r, 2);
  ASSERT_EQUAL(comp.base, SCALAR_BASE);
}

CTEST(suite, test_get_component_1) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  bool result = q.get_component(1, comp);
  ASSERT_EQUAL(result, true);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, I);
}

CTEST(suite, test_get_component_2) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  bool result = q.get_component(2, comp);
  ASSERT_EQUAL(result, true);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, J);
}

CTEST(suite, test_get_component_3) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  bool result = q.get_component(3, comp);
  ASSERT_EQUAL(result, true);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, K);
}
CTEST(suite, test_get_component_4) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  bool result = q.get_component(4, comp);
  ASSERT_EQUAL(result, false);
}
/*! @} */
/*! @} */

/*! @{ Test constructors of the quaternion object*/

CTEST(suite, test_constructor_0) {
  quaternion<real> q;
  ASSERT_EQUAL(q.r(), 0);
  ASSERT_EQUAL(q.x(), 1);
  ASSERT_EQUAL(q.y(), 1);
  ASSERT_EQUAL(q.z(), 1);
}

CTEST(suite, test_constructor_1) {
  quaternion<real> q(2, 3, 4, 5);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 3);
  ASSERT_EQUAL(q.y(), 4);
  ASSERT_EQUAL(q.z(), 5);
}

CTEST(suite, test_constructor_2) {
  real cs[4];
  cs[0] = 2;
  cs[1] = 3;
  cs[2] = 4;
  cs[3] = 5;
  quaternion<real> q(cs);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 3);
  ASSERT_EQUAL(q.y(), 4);
  ASSERT_EQUAL(q.z(), 5);
}

CTEST(suite, test_constructor_3) {
  quat_c<real> c1 = quat_c<real>(SCALAR_BASE, 2);
  quat_c<real> c2 = quat_c<real>(I, 3);
  quat_c<real> c3 = quat_c<real>(J, 4);
  quat_c<real> c4 = quat_c<real>(K, 5);
  quaternion<real> q(c1, c2, c3, c4);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 3);
  ASSERT_EQUAL(q.y(), 4);
  ASSERT_EQUAL(q.z(), 5);
}

CTEST(suite, test_constructor_4) {
  real c1 = 2;
  quat_c<real> c2 = quat_c<real>(I, 3);
  quat_c<real> c3 = quat_c<real>(J, 4);
  quat_c<real> c4 = quat_c<real>(K, 5);
  quaternion<real> q(c1, c2, c3, c4);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 3);
  ASSERT_EQUAL(q.y(), 4);
  ASSERT_EQUAL(q.z(), 5);
}

CTEST(suite, test_constructor_5) {
  real c1 = 2;
  real cs[3];
  cs[0] = 3;
  cs[1] = 4;
  cs[2] = 5;
  quaternion<real> q(c1, cs);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 3);
  ASSERT_EQUAL(q.y(), 4);
  ASSERT_EQUAL(q.z(), 5);
}

/*! @} */

/*! @{ Test vector operations */

CTEST(suite, test_vector_multiplication_0) {
  quaternion<real> q(2, 3, 4, 5);
  bool result = q.vector_multiplication(2);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 6);
  ASSERT_EQUAL(q.y(), 8);
  ASSERT_EQUAL(q.z(), 10);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_multiplication_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  bool result = q.vector_multiplication(v, 2);
  ASSERT_EQUAL(v[0], 6);
  ASSERT_EQUAL(v[1], 8);
  ASSERT_EQUAL(v[2], 10);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_multiplication_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  bool result = q.vector_multiplication(v, t);
  ASSERT_EQUAL(v[0], 6);
  ASSERT_EQUAL(v[1], 12);
  ASSERT_EQUAL(v[2], 5);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_addition_0) {
  quaternion<real> q(2, 3, 4, 5);
  bool result = q.vector_addition(2);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 5);
  ASSERT_EQUAL(q.y(), 6);
  ASSERT_EQUAL(q.z(), 7);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_addition_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  bool result = q.vector_addition(v, 2);
  ASSERT_EQUAL(v[0], 5);
  ASSERT_EQUAL(v[1], 6);
  ASSERT_EQUAL(v[2], 7);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_addition_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  bool result = q.vector_addition(v, t);
  ASSERT_EQUAL(v[0], 5);
  ASSERT_EQUAL(v[1], 7);
  ASSERT_EQUAL(v[2], 6);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_subtraction_0) {
  quaternion<real> q(2, 3, 4, 5);
  bool result = q.vector_subtraction(2);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 1);
  ASSERT_EQUAL(q.y(), 2);
  ASSERT_EQUAL(q.z(), 3);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_subtraction_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  bool result = q.vector_subtraction(v, 2);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_subtraction_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  bool result = q.vector_subtraction(v, t);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 1);
  ASSERT_EQUAL(v[2], 4);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_division_0) {
  quaternion<real> q(2, 4, 4, 6);
  bool result = q.vector_division(2);
  ASSERT_EQUAL(q.r(), 2);
  ASSERT_EQUAL(q.x(), 2);
  ASSERT_EQUAL(q.y(), 2);
  ASSERT_EQUAL(q.z(), 3);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_division_1) {
  quaternion<real> q(2, 2, 4, 6);
  real v[3];
  bool result = q.vector_division(v, 2);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_division_2) {
  quaternion<real> q(2, 4, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 4;
  t[2] = 1;
  bool result = q.vector_division(v, t);
  ASSERT_EQUAL(v[0], 2);
  ASSERT_EQUAL(v[1], 1);
  ASSERT_EQUAL(v[2], 5);
  ASSERT_EQUAL(result, true);
}
CTEST(suite, test_vector_division_3) {
  quaternion<real> q(2, 3, 4, 5);
  bool result = q.vector_division(static_cast<real>(0));
  ASSERT_EQUAL(result, false);
}
CTEST(suite, test_vector_division_4) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  bool result = q.vector_division(v, static_cast<real>(0));
  ASSERT_EQUAL(result, false);
}
CTEST(suite, test_vector_division_5) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 0;
  t[2] = 1;
  bool result = q.vector_division(v, t);
  ASSERT_EQUAL(result, false);
}
CTEST(suite, test_vector_dot_0) {
  quaternion<real> q(1, 9, 2, 7);
  real t[3];
  t[0] = 4;
  t[1] = 8;
  t[2] = 10;
  real result = q.vector_dot(t);
  ASSERT_EQUAL(result, 122);
}
CTEST(suite, test_vector_dot_1) {
  quaternion<real> q = quaternion<real>();
  real v[3];
  v[0] = 9;
  v[1] = 2;
  v[2] = 7;
  real t[3];
  t[0] = 4;
  t[1] = 8;
  t[2] = 10;
  real result = q.vector_dot(v, t);
  ASSERT_EQUAL(result, 122);
}
CTEST(suite, test_vector_cross_0) {
  quaternion<real> q(1, 2, 3, 4);
  real out[3];
  real t[3];
  t[0] = 5;
  t[1] = 6;
  t[2] = 7;
  bool result = q.vector_cross(out, t);
  ASSERT_EQUAL(result, true);
  ASSERT_EQUAL(out[0], static_cast<real>(-3));
  ASSERT_EQUAL(out[1], static_cast<real>(6));
  ASSERT_EQUAL(out[2], static_cast<real>(-3));
}
/*! @} */

/*! @{ Test hamilton product operation from Vince 2011 -
 * Quaternions for
 * Computer Graphics, p. 70 */
CTEST(suite, test_hamilton_product) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab = q_a.hamilton_product(q_b);
  // quaternion q_ab = q_a * q_b;
  ASSERT_EQUAL(q_ab.r(), static_cast<real>(-41));
  ASSERT_EQUAL(q_ab.x(), static_cast<real>(-4));
  ASSERT_EQUAL(q_ab.y(), static_cast<real>(9));
  ASSERT_EQUAL(q_ab.z(), static_cast<real>(-20));
}
/*! @} */
/*! @{ Test conjugate operation from Vince 2011, p. 70 */
CTEST(suite, test_conjugate_0) {
  quaternion<real> q_b(2, -2, 3, -4);
  quaternion<real> q_a = q_b.conjugate();
  ASSERT_EQUAL(q_a.r(), 2);
  ASSERT_EQUAL(q_a.x(), 2);
  ASSERT_EQUAL(q_a.y(), -3);
  ASSERT_EQUAL(q_a.z(), 4);
}
CTEST(suite, test_conjugate_1) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab = q_a.hamilton_product(q_b);
  quaternion<real> q_ab_conj = q_ab.conjugate();
  quaternion<real> q_a_conj_b_conj =
      q_b.conjugate().hamilton_product(q_a.conjugate());

  //
  ASSERT_EQUAL(q_ab_conj.r(), q_a_conj_b_conj.r());
  ASSERT_EQUAL(q_ab_conj.x(), q_a_conj_b_conj.x());
  ASSERT_EQUAL(q_ab_conj.y(), q_a_conj_b_conj.y());
  ASSERT_EQUAL(q_ab_conj.z(), q_a_conj_b_conj.z());
}

/*! @{ arithmetic ops for quaternions */
CTEST(suite, test_plus_operator) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab = q_a + q_b;
  ASSERT_EQUAL(q_ab.r(), 3);
  ASSERT_EQUAL(q_ab.x(), -4);
  ASSERT_EQUAL(q_ab.y(), 8);
  ASSERT_EQUAL(q_ab.z(), -10);
}

CTEST(suite, test_minus_operator) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab = q_a - q_b;
  ASSERT_EQUAL(q_ab.r(), 1);
  ASSERT_EQUAL(q_ab.x(), 0);
  ASSERT_EQUAL(q_ab.y(), -2);
  ASSERT_EQUAL(q_ab.z(), 2);
}

CTEST(suite, test_star_operator_0) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab = q_a * q_b;
  ASSERT_EQUAL(q_ab.r(), static_cast<real>(-41));
  ASSERT_EQUAL(q_ab.x(), static_cast<real>(-4));
  ASSERT_EQUAL(q_ab.y(), static_cast<real>(9));
  ASSERT_EQUAL(q_ab.z(), static_cast<real>(-20));
}

CTEST(suite, test_star_operator_1) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_ab = q_a * 2;
  ASSERT_EQUAL(q_ab.r(), static_cast<real>(4));
  ASSERT_EQUAL(q_ab.x(), static_cast<real>(-4));
  ASSERT_EQUAL(q_ab.y(), static_cast<real>(6));
  ASSERT_EQUAL(q_ab.z(), static_cast<real>(-8));
}
/*! @} */

/*! @{ Test determinant of quaternion */
CTEST(suite, test_determinant) {
  quaternion<real> q_a(2, -2, 3, -4);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(q_a.determinant(), static_cast<real>(33));
}

/*! Test determinant of quaternion */
CTEST(suite, test_det) {
  quaternion<real> q_a(2, -2, 3, -4);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(q_a.det(), static_cast<real>(33));
}
/*! @} */

/*! @{ Test magnitude of quaternion */
CTEST(suite, test_magnitude) {
  quaternion<real> q_a(2, -2, 3, -4);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(q_a.magnitude(), static_cast<real>(sqrt(33)));
}

/*! Test norm of quaternion */
CTEST(suite, test_norm) {
  quaternion<real> q_a(2, -2, 3, -4);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(q_a.norm(), static_cast<real>(sqrt(33)));
}

/*! @} */

/*! @{ Test exponent of quaternion */
CTEST(suite, test_squared) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_a2 = q_a.squared();
  ASSERT_EQUAL(q_a2.r(), static_cast<real>(-25));
  ASSERT_EQUAL(q_a2.x(), static_cast<real>(-8));
  ASSERT_EQUAL(q_a2.y(), static_cast<real>(12));
  ASSERT_EQUAL(q_a2.z(), static_cast<real>(-16));
}

CTEST(suite, test_power) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_a2 = q_a.power(2);
  ASSERT_EQUAL(q_a2.r(), static_cast<real>(-25));
  ASSERT_EQUAL(q_a2.x(), static_cast<real>(-8));
  ASSERT_EQUAL(q_a2.y(), static_cast<real>(12));
  ASSERT_EQUAL(q_a2.z(), static_cast<real>(-16));
}

/*! @} */

/*! @{ Test inverse of quaternion */
CTEST(suite, test_inversed) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> inv = q_a.inversed();
  ASSERT_EQUAL(inv.r(), static_cast<real>(static_cast<real>(1.0 / 33) * 2));
  ASSERT_EQUAL(inv.x(), static_cast<real>(static_cast<real>(1.0 / 33) * 2));
  ASSERT_EQUAL(inv.y(), static_cast<real>(static_cast<real>(1.0 / 33) * -3));
  ASSERT_EQUAL(inv.z(), static_cast<real>(static_cast<real>(1.0 / 33) * 4));
}
/*! @} */

/*! @{ Test normalization method of quaternion */
CTEST(suite, test_normalized) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> n_q = q_a.normalized();
  ASSERT_EQUAL(n_q.r(),
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * 2));
  ASSERT_EQUAL(n_q.x(),
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * -2));
  ASSERT_EQUAL(n_q.y(),
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * 3));
  ASSERT_EQUAL(n_q.z(),
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * -4));
}
/*! @} */
