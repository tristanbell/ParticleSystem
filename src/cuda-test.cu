#include <cuda.h>
#include <iostream>

__host__ void test() {
  float a = 12.;
  float b = 3.;
  float c = a * b;
  std::cout << c << std::endl;
}
