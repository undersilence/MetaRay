#include <iostream>
#include <memory>

#include "MTR/math_defs.h"
#include "MTR/softras/render.hpp"

typedef A2V Vertex;

struct Test_V2F {
  vec2f tex_coords;
};

int main() {
  int test_width = 500, test_height = 500;
  auto shader = std::make_shared<SoftShader>();
  auto raster = std::make_shared<SoftRaster<decltype(shader->arr2vert_cls),
                                            decltype(shader->vert2frag_cls)>>(
      test_width, test_height);
  raster->test();

  raster->set_shader(shader);
  raster->set_project(
      perspective(60.0f, test_width / (float)test_height, 0.1f, 50.0f));
  std::vector<Vertex> vertex_data;
  Vertex a, b, c;
  a.position << 0.5f, 0.5f, 1.5f;
  b.position << -0.5f, 0.5f, 2.0f;
  c.position << 0.5f, -0.5f, 2.5f;
  vertex_data.insert(vertex_data.end(), {a, b, c});

  auto buffer_id = raster->load_array(vertex_data);
  raster->draw_arrays(buffer_id);

  auto &frame_buf = raster->frame_buffer();

  FILE *fp = fopen("test.ppm", "wb");
  (void)fprintf(fp, "P6\n%d %d\n255\n", test_width, test_height);
  for (auto i = 0; i < test_height * test_width; ++i) {
    static unsigned char color[3];
    color[0] = (unsigned char)(255 * clamp(0, 1, frame_buf[i].x()));
    color[1] = (unsigned char)(255 * clamp(0, 1, frame_buf[i].y()));
    color[2] = (unsigned char)(255 * clamp(0, 1, frame_buf[i].z()));
    fwrite(color, 1, 3, fp);
  }
  fclose(fp);

  printf("hello from MTR\n");
  return 0;
}