# quaternion
Consisted, tested, header only quaternion template for C++11


The current api is quite clean and consistent. It should be fairly easy to
adapt it.

All methods are qualified with `const`. We never throw exceptions `throw`,
`new`, and `delete` does not occur anywhere in the code base.
All methods are structured as side effects. All output values are always the
last argument of functions. See usage section for usage examples.

We have about 45 tests that cover all the methods.


# Including

We provide two versions of the library:

- The all including `quaternion.hpp` which contains the declarations and
  implementations for the methods of the quaternion. This is recommended for
  most use cases.

- The `quaternion.h` and `quaternion.cpp`. The first one contains just the
  declarations and the second one contains implementations. Note that as
  stated in [isocpp
  faq](https://isocpp.org/wiki/faq/templates#separate-template-class-defn-from-decl)
  before using templates in this fashion, you have to declare the template
  class with the used type somewhere before usage. You need to have something
  like:

```c++
// myfile.cpp 
#include "quaternion.cpp"

using namespace quat11;

template class quaternion<float>;

void myfunc(){
    quaternion<float> q(1,2,3,5);
    // other stuff ...
}

```

# Usage

The usage examples are adapted from the unit tests.

- Take scalar part from quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;
typedef float real;

int maint(){
  quaternion<real> q;
  real t = static_cast<real>(4561);
  auto res = q.scalar(t);

  ASSERT_EQUAL(t, 0);
  ASSERT_EQUAL(res, SUCCESS);
  return 0;
}

```

- Take real vector part from quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
  quaternion<real> q;
  real vs[3];
  auto res = q.vector(vs);
  real comp[3] = {1, 1, 1};
  ASSERT_EQUAL(res, SUCCESS);
  ASSERT_EQUAL(vs[0], comp[0]);
  ASSERT_EQUAL(vs[1], comp[1]);
  ASSERT_EQUAL(vs[2], comp[2]);

  return 0;
}

```

- Take component from quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){

  quaternion<real> q(2, 3, 3, 3);
  quat_c<real> comp;
  auto result = q.get_component(0, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 2);
  ASSERT_EQUAL(comp.base, SCALAR_BASE);

  result = q.get_component(1, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, I);

  result = q.get_component(2, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, J);

  result = q.get_component(3, comp);
  ASSERT_EQUAL(result, SUCCESS);
  ASSERT_EQUAL(comp.r, 3);
  ASSERT_EQUAL(comp.base, K);

  result = q.get_component(4, comp);
  ASSERT_EQUAL(result, ARG_ERROR);

  return 0;
}
```

- Ways to construct a quaternion

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){

  // empty constructor (0, 1, 1, 1) as real coefficients
  quaternion<real> q;

  quaternion<real> q2(2, 3, 4, 5);

  real cs[4];
  cs[0] = 2;
  cs[1] = 3;
  cs[2] = 4;
  cs[3] = 5;
  quaternion<real> q3(cs);

  quat_c<real> c1 = quat_c<real>(SCALAR_BASE, 2);
  quat_c<real> c2 = quat_c<real>(I, 3);
  quat_c<real> c3 = quat_c<real>(J, 4);
  quat_c<real> c4 = quat_c<real>(K, 5);
  quaternion<real> q4(c1, c2, c3, c4);

  c1 = 2;
  c2 = quat_c<real>(I, 3);
  c3 = quat_c<real>(J, 4);
  c4 = quat_c<real>(K, 5);
  quaternion<real> q5(c1, c2, c3, c4);

  c1 = 2; // scalar part
  real cs1[3]; // vector real part
  cs1[0] = 3;
  cs1[1] = 4;
  cs1[2] = 5;
  quaternion<real> q(c1, cs1);

  return 0;
}
```

- Add two quaternions together

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_out;
  q_a.add(q_b, q_out);

  real s = static_cast<real>(1);
  auto res = q_out.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_out.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, 3);
  ASSERT_EQUAL(vec[0], -4);
  ASSERT_EQUAL(vec[1], 8);
  ASSERT_EQUAL(vec[2], -10);


  return 0;
}
```

- Subtract quaternion from another together

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_out;
  q_a.subtract(q_b, q_out);

  real s = static_cast<real>(1);
  auto res = q_out.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_out.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, 1);
  ASSERT_EQUAL(vec[0], 0);
  ASSERT_EQUAL(vec[1], -2);
  ASSERT_EQUAL(vec[2], 2);

  return 0;
}
```

- Hamilton product between two quaternions

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
  quaternion<real> q_a(2, -2, 3, -4);
  quaternion<real> q_b(1, -2, 5, -6);
  quaternion<real> q_out;
  q_a.product(q_b, q_out);

  real s = static_cast<real>(1);
  auto res = q_out.scalar(s);
  ASSERT_EQUAL(res, SUCCESS);

  real vec[3];
  res = q_out.vector(vec);
  ASSERT_EQUAL(res, SUCCESS);

  ASSERT_EQUAL(s, static_cast<real>(-41));
  ASSERT_EQUAL(vec[0], static_cast<real>(-4));
  ASSERT_EQUAL(vec[1], static_cast<real>(9));
  ASSERT_EQUAL(vec[2], static_cast<real>(-20));

  return 0;
}
```

- Determinant of quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
  quaternion<real> q_a(2, -2, 3, -4);
  real dv = static_cast<real>(5);
  q_a.determinant(dv);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(dv, static_cast<real>(33));

  return 0;
}
```

- Magnitude/norm of quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){

  quaternion<real> q_a(2, -2, 3, -4);
  real dv = static_cast<real>(5);
  q_a.magnitude(dv);
  // 4 + 4 + 9 + 16
  ASSERT_EQUAL(dv, static_cast<real>(sqrt(33)));

  return 0;
}
```

- Take power of a quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){

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

  return 0;
}
```

- Take inverse of a quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
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

  return 0;
}
```

- Normalize a quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){
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

  return 0;
}
```

- Conjugate a quaternion:

```c++
// myfile.cpp 
#include "quaternion.hpp"

using namespace quat11;

typedef float real;

int maint(){

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

  return 0;
}
```

Lastly we provide three macros to wrap method calls.

- `CHECK`: returns a boolean if the call outputs `SUCCESS` flag
- `INFO`: returns `QUATERNION_FLAG` flag but shows out the file and line
  information if the call outputs an error flag.
- `INFO_VERBOSE`: returns `QUATERNION_FLAG` flag but shows the file and line
  information if the call outputs an error flag, also shows success message.

Here is a usage example:

```c++
// myfile.cpp 
#include <ostream>
#include "quaternion.hpp"

int main(){
  quaternion<real> q_b(2, -2, 3, -4);
  quaternion<real> q_out;
  bool result = CHECK(q_b.conjugate(q_out));
  std::cout << result << std::endl;
  // true

  quat_c<real> comp;
  auto res = INFO(q.get_component(4, comp));
  // prints to cout: ARG_ERROR :: myfile.cpp :: 10

  std::cout << res << std::endl;
  // INDEX_ERROR

  return 0;
}
```
 
