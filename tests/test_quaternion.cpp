// test file for quaternion
#include "../quaternion.hpp"
#include <ctest.h>

using namespace quat;

CTEST(suite, test_scalar1) {
  quaternion q;
  ASSERT_EQUAL(q.scalar(), 0);
}
CTEST(suite, test_vector1) {
  quaternion q;
  real vs[3];
  q.vector(vs);
  real comp[3] = {1, 1, 1};
  ASSERT_EQUAL(vs[0], comp[0]);
  ASSERT_EQUAL(vs[1], comp[1]);
  ASSERT_EQUAL(vs[2], comp[2]);
};
