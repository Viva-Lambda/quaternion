# quaternion
Simple, tested, header only quaternion library for C++98

It should be fairly easy to use an integrate into existing projects.
Complex bases are implemented as enums. One can access components of a
quaternion as:

```c++
#include <ostream>
#include "quaternion.hpp"

int main(){
  quaternion<float> q(4, 3, 4, 1);
  quat_c<float> component1;
  q.get_component(0, component1);

  std::cout << component1 << std::endl;
  // would give you SCALAR_BASE::4

  quat_c<float> component2;
  q.get_component(2, component2);

  std::cout << component2 << std::endl;
  // would give you J_BASE::4
  return 0;
}
```

Testing is more or less done now. Besides printing functions all methods of
quaternion object had been tested
