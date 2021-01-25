#include "MTR/math_defs.h"
#include "MTR/softras/render.h"
#include <iostream>
#include <memory>

int main() {
  auto raster = std::make_shared<SoftRaster>(800, 600);
  raster->test();
  printf("hello from MTR\n");
  return 0;
}