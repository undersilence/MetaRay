#include "MTR/softras/render.h"

SoftRaster::SoftRaster(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

void SoftRaster::draw_arrays(SoftRaster::Primitive mode) {
  printf("hello from drawArrays.\n");
  // draw triangles [0, pos_buf)
  int size = 3;
  int stride = 3;

  auto mvp = project * view * model;

  if (mode == Primitive::Triangle) {
    auto &curr_buf = arr_bufs[0];
    for (int i = size; i < curr_buf.size(); i += 3 * stride) {
      A2V a2v_a{}, a2v_b{}, a2v_c{};
      V2F v2f_a{}, v2f_b{}, v2f_c{};

      // only configure positions for test
      vec3f a();
      vec3f b();
      vec3f c();

      a2v_a.position << curr_buf[i - 3], curr_buf[i - 2], curr_buf[i - 1];
      a2v_b.position << curr_buf[i - 3 + size], curr_buf[i - 2 + size],
          curr_buf[i - 1 + size];
      a2v_c.position << curr_buf[i - 3 + 2 * size], curr_buf[i - 2 + 2 * size],
          curr_buf[i - 1 + 2 * size];

      a2v_a.project = a2v_b.project = a2v_c.project = project;
      a2v_a.model = a2v_b.model = a2v_c.model = model;
      a2v_a.view = a2v_b.view = a2v_c.view = view;

      // vertex position in screen space
      vec4f v[] = {vertex_shader(a2v_a, v2f_a), vertex_shader(a2v_b, v2f_b),
                   vertex_shader(a2v_c, v2f_c)};

      // Homogeneous division
      for (auto &vert : v) {
        vert.x() /= vert.w();
        vert.y() /= vert.w();
        vert.z() /= vert.w();
      }

      // Viewport transformation
      for (auto &vert : v) {
        vert.x() = 0.5 * width * (vert.x() + 1.0);
        vert.y() = 0.5 * height * (vert.y() + 1.0);
      }

      v2f_a.screenpos = v[0];
      v2f_b.screenpos = v[0];
      v2f_c.screenpos = v[0];

      // scan triangles
      rasterize_triangle(v2f_a, v2f_b, v2f_c);
    }
  }
}

void SoftRaster::rasterize_triangle(V2F a, V2F b, V2F c) {}

void SoftRaster::draw_elements() {}
void SoftRaster::set_vertex_shader(
    const std::function<vec4f(A2V &, V2F &)> &_vertex_shader) {
  vertex_shader = _vertex_shader;
}
void SoftRaster::set_fragment_shader(
    const std::function<vec3f(V2F &)> &_fragment_shader) {
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

SoftRaster::ind_buf_id SoftRaster::load_indices(const std::vector<int> &inds) {
  auto id = get_next_id();
  ind_bufs.emplace(id, inds);
  return {id};
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y,
                                                            vec4f *v) {
  float c1 =
      (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y +
       v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
      (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() +
       v[1].x() * v[2].y() - v[2].x() * v[1].y());
  float c2 =
      (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y +
       v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
      (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() +
       v[2].x() * v[0].y() - v[0].x() * v[2].y());
  float c3 =
      (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y +
       v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
      (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() +
       v[0].x() * v[1].y() - v[1].x() * v[0].y());
  return {c1, c2, c3};
}