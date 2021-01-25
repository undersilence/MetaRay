#include "MTR/softras/render.h"

SoftRaster::SoftRaster(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

void SoftRaster::draw_arrays(SoftRaster::Primitive mode) {
  printf("hello from drawArrays.\n");
  // draw triangles [0, pos_buf)
  int size = 3;
  int stride = 0;

  auto mvp = model * view * project;

  if (mode == Primitive::Triangle) {
    auto &curr_buf = arr_bufs[0];
    for (int i = size; i < curr_buf.size(); i += 3 * size) {
      vertex_shader_payload a2v_a{}, a2v_b{}, a2v_c{};
      fragment_shader_payload v2f_a{}, v2f_b{}, v2f_c{};

      // only configure positions for test
      vec3f a(curr_buf[i - 3], curr_buf[i - 2], curr_buf[i - 1]);
      vec3f b(curr_buf[i - 3 + size], curr_buf[i - 2 + size],
              curr_buf[i - 1 + size]);
      vec3f c(curr_buf[i - 3 + 2 * size], curr_buf[i - 2 + 2 * size],
              curr_buf[i - 1 + 2 * size]);
      a2v_a.position = a, a2v_b.position = b, a2v_c.position = c;

      // vertex position in view space
      vec4f pos_a = vertex_shader(a2v_a, v2f_a);
      vec4f pos_b = vertex_shader(a2v_b, v2f_b);
      vec4f pos_c = vertex_shader(a2v_c, v2f_c);
    }
  }
}

void SoftRaster::draw_elements() {}
void SoftRaster::set_vertex_shader(
    const std::function<vec4f(vertex_shader_payload &,
                              fragment_shader_payload &)> &_vertex_shader) {
  vertex_shader = _vertex_shader;
}
void SoftRaster::set_fragment_shader(
    const std::function<vec3f(fragment_shader_payload &)> &_fragment_shader) {
  fragment_shader = _fragment_shader;
}

int SoftRaster::get_next_id() { return next_id++; }

void SoftRaster::set_model(const mat4f &m) { model = m; }

void SoftRaster::set_view(const mat4f &v) { view = v; }

void SoftRaster::set_project(const mat4f &p) { project = p; }

void SoftRaster::set_pixel(const vec2i &coord, const vec3f &color) {
  frame_buf[width * coord.y() + coord.x()] = color;
}

SoftRaster::arr_buf_id SoftRaster::load_array(const std::vector<float> &arr) {
  auto id = get_next_id();
  arr_bufs.emplace(id, arr);
  return {id};
}

SoftRaster::ind_buf_id
SoftRaster::load_indices(const std::vector<int> &indices) {
  auto id = get_next_id();
  ind_bufs.emplace(id, indices);
  return {id};
}