#include "MTR/math_defs.h"
#include "MTR/softras/render.h"
#include <iostream>
#include <memory>

int main() {
  auto raster = std::make_shared<SoftRaster>(800, 600);
  raster->test();

  raster->set_project(perspective(60, )) raster->load_array({0.5f, 0.5f, 1.0f, -0.5f, 0.5f, 0.5f, 0.5f, -0.5f, 0.5f});
  raster->draw_arrays();
  printf("hello from MTR\n");
  return 0;
}