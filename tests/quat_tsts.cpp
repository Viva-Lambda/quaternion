#include <ctest.h>

using namespace quat11;

/*! @{
  Test accessors of quaternion object. These functions help
  us to access
  real parts, that is the scalar and the vector part of the
  quaternion object.
 */

CTEST(suite, test_scalar1) {
  quaternion<real> q;
  real t = static_cast<real>(4561);
  auto res = q.scalar(t);

  ASSERT_EQUAL(t, 0);
  ASSERT_EQUAL(res, SUCCESS);
}
CTEST(suite, test_vector1) {
  quaternion<real> q;
  real vs[3];
  auto res = q.vector(vs);
  real comp[3] = {1, 1, 1};
  ASSERT_EQUAL(res, SUCCESS);
  ASSERT_EQUAL(vs[0], comp[0]);
  ASSERT_EQUAL(vs[1], comp[1]);
  ASSERT_EQUAL(vs[2], comp[2]);
};

/*! @{ test component accessors */
CTEST(suite, test_get_component_0) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(0, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 2);
  ASSERT_EQUAL(comp.base, SCALAR_BASE);
}

CTEST(suite, test_get_component_1) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(1, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, I);
}

CTEST(suite, test_get_component_2) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(2, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, J);
}

CTEST(suite, test_get_component_3) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(3, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, K);
}
CTEST(suite, test_get_component_4) {
  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(4, comp);
  ASSERT_EQUAL(result, ARG_ERROR);
}
/*! @} */
/*! @} */

/*! @{ Test constructors of the quaternion object*/

CTEST(suite, test_constructor_0) {
  quaternion<real> q;
  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 0);
  ASSERT_EQUAL(vec[0], 1);
  ASSERT_EQUAL(vec[1], 1);
  ASSERT_EQUAL(vec[2], 1);
}

CTEST(suite, test_constructor_1) {
  quaternion<real> q(2, 3, 4, 5);

  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(vec[0], 3);
  ASSERT_EQUAL(vec[1], 4);
  ASSERT_EQUAL(vec[2], 5);
}

CTEST(suite, test_constructor_2) {
  real cs[4];
  cs[0] = 2;
  cs[1] = 3;
  cs[2] = 4;
  cs[3] = 5;
  quaternion<real> q(cs);

  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(vec[0], 3);
  ASSERT_EQUAL(vec[1], 4);
  ASSERT_EQUAL(vec[2], 5);
}

CTEST(suite, test_constructor_3) {
  quat_c<real> c1 = quat_c<real>(SCALAR_BASE, 2);
  quat_c<real> c2 = quat_c<real>(I, 3);
  quat_c<real> c3 = quat_c<real>(J, 4);
  quat_c<real> c4 = quat_c<real>(K, 5);
  quaternion<real> q(c1, c2, c3, c4);

  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(vec[0], 3);
  ASSERT_EQUAL(vec[1], 4);
  ASSERT_EQUAL(vec[2], 5);
}

CTEST(suite, test_constructor_4) {
  real c1 = 2;
  quat_c<real> c2 = quat_c<real>(I, 3);
  quat_c<real> c3 = quat_c<real>(J, 4);
  quat_c<real> c4 = quat_c<real>(K, 5);
  quaternion<real> q(c1, c2, c3, c4);

  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(vec[0], 3);
  ASSERT_EQUAL(vec[1], 4);
  ASSERT_EQUAL(vec[2], 5);
}

CTEST(suite, test_constructor_5) {
  real c1 = 2;
  real cs[3];
  cs[0] = 3;
  cs[1] = 4;
  cs[2] = 5;
  quaternion<real> q(c1, cs);
  real s = static_cast<real>(1);
  auto res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(vec[0], 3);
  ASSERT_EQUAL(vec[1], 4);
  ASSERT_EQUAL(vec[2], 5);
}

/*! @} */

/*! @{ Test vector operations */

CTEST(suite, test_vector_multiplication_0) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];

  auto res = q.vector_multiplication(2, v);
  ASSERT_EQUAL(res, SUCCESS);

  real s = static_cast<real>(1);
  res = q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  //
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(v[0], 6);
  ASSERT_EQUAL(v[1], 8);
  ASSERT_EQUAL(v[2], 10);
}
CTEST(suite, test_vector_multiplication_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  auto res = q.vector_multiplication(2, v);
  ASSERT_EQUAL(res, SUCCESS);
  //
  ASSERT_EQUAL(v[0], 6);
  ASSERT_EQUAL(v[1], 8);
  ASSERT_EQUAL(v[2], 10);
}
CTEST(suite, test_vector_multiplication_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  auto result = q.vector_multiplication(t, v);
  ASSERT_EQUAL(v[0], 6);
  ASSERT_EQUAL(v[1], 12);
  ASSERT_EQUAL(v[2], 5);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_addition_0) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  auto result = q.vector_addition(2, v);
  real s = static_cast<real>(10);
  q.scalar(s);
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(v[0], 5);
  ASSERT_EQUAL(v[1], 6);
  ASSERT_EQUAL(v[2], 7);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_addition_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  auto result = q.vector_addition(2, v);
  ASSERT_EQUAL(v[0], 5);
  ASSERT_EQUAL(v[1], 6);
  ASSERT_EQUAL(v[2], 7);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_addition_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  auto result = q.vector_addition(t, v);
  ASSERT_EQUAL(v[0], 5);
  ASSERT_EQUAL(v[1], 7);
  ASSERT_EQUAL(v[2], 6);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_subtraction_0) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  q.vector(v);
  auto result = q.vector_subtraction(2, v);
  real s = static_cast<real>(22);
  q.scalar(s);
  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_subtraction_1) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  auto result = q.vector_subtraction(2, v);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_subtraction_2) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 3;
  t[2] = 1;
  auto result = q.vector_subtraction(t, v);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 1);
  ASSERT_EQUAL(v[2], 4);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_division_0) {
  quaternion<real> q(2, 4, 4, 6);
  real v[3];
  q.vector(v);
  auto result = q.vector_division(2, v);

  real s = static_cast<real>(1);
  result = q.scalar(s);
  ASSERT_EQUAL(result, SUCCESS);

  ASSERT_EQUAL(s, 2);
  ASSERT_EQUAL(v[0], 2);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_division_1) {
  quaternion<real> q(2, 2, 4, 6);
  real v[3];
  auto result = q.vector_division(2, v);
  ASSERT_EQUAL(v[0], 1);
  ASSERT_EQUAL(v[1], 2);
  ASSERT_EQUAL(v[2], 3);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_division_2) {
  quaternion<real> q(2, 4, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 4;
  t[2] = 1;
  auto result = q.vector_division(t, v);
  ASSERT_EQUAL(v[0], 2);
  ASSERT_EQUAL(v[1], 1);
  ASSERT_EQUAL(v[2], 5);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_division_3) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  q.vector(v);
  auto result = q.vector_division(static_cast<real>(0), v);
  ASSERT_EQUAL(result, ARG_ERROR);
}
CTEST(suite, test_vector_division_4) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  auto result = q.vector_division(static_cast<real>(0), v);
  ASSERT_EQUAL(result, ARG_ERROR);
}
CTEST(suite, test_vector_division_5) {
  quaternion<real> q(2, 3, 4, 5);
  real v[3];
  real t[3];
  t[0] = 2;
  t[1] = 0;
  t[2] = 1;
  auto result = q.vector_division(t, v);
  ASSERT_EQUAL(result, ARG_ERROR);
}
CTEST(suite, test_vector_dot_0) {
  quaternion<real> q(1, 9, 2, 7);
  real t[3];
  t[0] = 4;
  t[1] = 8;
  t[2] = 10;
  real out;
  auto result = q.vector_dot(t, out);
  ASSERT_EQUAL(out, 122);
  ASSERT_EQUAL(result, SUCCESS);
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
  real s = static_cast<real>(55156);
  auto result = q.vector_dot(v, t, s);
  ASSERT_EQUAL(s, 122);
  ASSERT_EQUAL(result, SUCCESS);
}
CTEST(suite, test_vector_cross_0) {
  quaternion<real> q(1, 2, 3, 4);
  real out[3];
  real t[3];
  t[0] = 5;
  t[1] = 6;
  t[2] = 7;
  auto result = q.vector_cross(t, out);
  ASSERT_EQUAL(result, SUCCESS);
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
  quaternion<real> q_out;
  q_a.hamilton_product(q_b, q_out);
  // quaternion q_ab = q_a * q_b;

  real s = static_cast<real>(0);
  auto res = q_out.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real v[3];
  res = q_out.vector(v);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(-41));
  ASSERT_EQUAL(v[0], static_cast<real>(-4));
  ASSERT_EQUAL(v[1], static_cast<real>(9));
  ASSERT_EQUAL(v[2], static_cast<real>(-20));
}
/*! @} */
/*! @{ Test conjugate operation from Vince 2011, p. 70 */
CTEST(suite, test_conjugate_0) {
  quaternion<real> q_b(2, -2, 3, -4);
  quaternion<real> q_out;
  q_b.conjugate(q_out);

  real s = static_cast<real>(0);
  auto res = q_out.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real v[3];
  res = q_out.vector(v);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(2));
  ASSERT_EQUAL(v[0], static_cast<real>(2));
  ASSERT_EQUAL(v[1], static_cast<real>(-3));
  ASSERT_EQUAL(v[2], static_cast<real>(4));
}
CTEST(suite, test_conjugate_1) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab_prod;
  q_a.hamilton_product(q_b, q_ab_prod);

  quaternion<real> q_out1;
  q_ab_prod.conjugate(q_out1);

  //
  quaternion<real> q_b_conj;
  q_b.conjugate(q_b_conj);

  quaternion<real> q_a_conj;
  q_a.conjugate(q_a_conj);

  quaternion<real> q_out2;
  q_b_conj.hamilton_product(q_a_conj, q_out2);

  //
  real s = static_cast<real>(0);
  real sq = static_cast<real>(0);

  q_out1.scalar(s);
  q_out2.scalar(sq);

  //
  real v[3];
  real vq[3];
  q_out1.vector(v);
  q_out2.vector(vq);

  ASSERT_EQUAL(s, sq);
  ASSERT_EQUAL(v[0], vq[0]);
  ASSERT_EQUAL(v[1], vq[1]);
  ASSERT_EQUAL(v[2], vq[2]);
}

/*! @{ arithmetic ops for quaternions */
CTEST(suite, test_add_operator) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab;
  q_a.add(q_b, q_ab);

  real s = static_cast<real>(1);
  auto res = q_ab.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_ab.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, 3);
  ASSERT_EQUAL(vec[0], -4);
  ASSERT_EQUAL(vec[1], 8);
  ASSERT_EQUAL(vec[2], -10);
}
CTEST(suite, test_subtract_operator) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab;
  q_a.subtract(q_b, q_ab);

  real s = static_cast<real>(1);
  auto res = q_ab.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_ab.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, 1);
  ASSERT_EQUAL(vec[0], 0);
  ASSERT_EQUAL(vec[1], -2);
  ASSERT_EQUAL(vec[2], 2);
}
CTEST(suite, test_product_operator_0) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_ab;
  q_a.product(q_b, q_ab);

  real s = static_cast<real>(1);
  auto res = q_ab.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_ab.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(-41));
  ASSERT_EQUAL(vec[0], static_cast<real>(-4));
  ASSERT_EQUAL(vec[1], static_cast<real>(9));
  ASSERT_EQUAL(vec[2], static_cast<real>(-20));
}
CTEST(suite, test_product_operator_1) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_ab;
  q_a.product(2, q_ab);

  real s = static_cast<real>(1);
  auto res = q_ab.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_ab.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, 4);
  ASSERT_EQUAL(vec[0], -4);
  ASSERT_EQUAL(vec[1], 6);
  ASSERT_EQUAL(vec[2], -8);
}
/*! @} */

/*! @{ Test determinant of quaternion */
CTEST(suite, test_determinant) {
  quaternion<real> q_a(2, -2, 3, -4);
  real dv = static_cast<real>(156);
  q_a.determinant(dv);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(dv, static_cast<real>(33));
}

/*! Test determinant of quaternion */
CTEST(suite, test_det) {
  quaternion<real> q_a(2, -2, 3, -4);
  real dv = static_cast<real>(5);
  q_a.det(dv);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(dv, static_cast<real>(33));
}
/*! @} */

/*! @{ Test magnitude of quaternion */
CTEST(suite, test_magnitude) {
  quaternion<real> q_a(2, -2, 3, -4);
  real dv = static_cast<real>(5);
  q_a.magnitude(dv);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(dv, static_cast<real>(sqrt(33)));
}

/*! Test norm of quaternion */
CTEST(suite, test_norm) {
  quaternion<real> q_a(2, -2, 3, -4);
  real n = static_cast<real>(0);
  auto res = q_a.norm(n);
  ASSERT_EQUAL(res, SUCCESS);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(n, static_cast<real>(sqrt(33)));
}

/*! @} */

/*! @{ Test exponent of quaternion */
CTEST(suite, test_squared) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_a2;
  q_a.squared(q_a2);

  //
  real s = static_cast<real>(1);
  auto res = q_a2.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_a2.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, -25);
  ASSERT_EQUAL(vec[0], -8);
  ASSERT_EQUAL(vec[1], 12);
  ASSERT_EQUAL(vec[2], -16);
}

CTEST(suite, test_power) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_a2;
  q_a.power(2, q_a2);

  real s = static_cast<real>(1);
  auto res = q_a2.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_a2.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, -25); // might be -25
  ASSERT_EQUAL(vec[0], -8);
  ASSERT_EQUAL(vec[1], 12);
  ASSERT_EQUAL(vec[2], -16);
}

/*! @} */

/*! @{ Test inverse of quaternion */
CTEST(suite, test_inversed) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> inv;
  q_a.inversed(inv);

  real s = static_cast<real>(1);
  auto res = inv.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = inv.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(static_cast<real>(1.0 / 33) * 2));
  ASSERT_EQUAL(vec[0], static_cast<real>(static_cast<real>(1.0 / 33) * 2));
  ASSERT_EQUAL(vec[1], static_cast<real>(static_cast<real>(1.0 / 33) * -3));
  ASSERT_EQUAL(vec[2], static_cast<real>(static_cast<real>(1.0 / 33) * 4));
}
/*! @} */

/*! @{ Test normalization method of quaternion */
CTEST(suite, test_normalized) {
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> n_q;
  q_a.normalized(n_q);

  real s = static_cast<real>(1);
  auto res = n_q.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = n_q.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * 2));
  ASSERT_EQUAL(vec[0],
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * -2));
  ASSERT_EQUAL(vec[1],
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * 3));
  ASSERT_EQUAL(vec[2],
               static_cast<real>(static_cast<real>(1.0 / sqrt(33)) * -4));
}
/*! @} */
