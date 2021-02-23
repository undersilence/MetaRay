#pragma once

#include "MTR/core.h"
#include "MTR/math_defs.h"
#include "MTR/shader.hpp"

namespace mtr {
class SoftRaster {
 public:
  enum Primitive { Line, Triangle };
  enum Buffer { Color = 1, Depth = 2 };
  struct arr_buf_id {
    int arr_id = 0;
  };
  struct ind_buf_id {
    int ind_id = 0;
  };

  explicit SoftRaster(int w, int h);

  template <class TArr2Vert, class TShader>
  void draw_arrays(const std::vector<TArr2Vert> &vertex_arr,
                   const std::shared_ptr<TShader> &shader,
                   Primitive mode = Primitive::Triangle);

  // void draw_elements(ind_buf_id buf_id, Primitive mode =
  // Primitive::Triangle);

  void clear(int buffer_mask);

  // void set_shader(const std::shared_ptr<SoftShader>& shader);
  // void set_vertex_shader(const std::function<vec4f(A2V &, V2F &)>
  // &VertexShader); void set_fragment_shader(const std::function<vec4f(V2F &)>
  // &FragmentShader);
  // void set_shader(const std::shared_ptr<IShader> &_shader);

  // arr_buf_id load_array(const std::vector<float> &arr);
  // arr_buf_id load_array(const std::vector<TApp2Vert> &vertex_arr);
  // ind_buf_id load_indices(const std::vector<int> &indices);
  int get_next_id();
  int get_index(int x, int y);

  void set_model(const mat4f &m);
  void set_view(const mat4f &v);
  void set_project(const mat4f &p);
  void set_pixel(const vec2i &coord, const vec4f &color);

  std::vector<uchar> &encode_frame_buffer();
  std::vector<vec4f> &frame_buffer() { return frame_buf; }
  std::vector<float> &depth_buffer() { return depth_buf; }

  void test() { printf("hello from SoftRaster.\n"); }

 protected:
  int width, height;
  int next_id = 0;
  // std::shared_ptr<SoftShader> shader;
  // std::shared_ptr<IShader> shader;

  void draw_line(vec3f begin, vec3f end);

  template <class TVert2Frag, class TShader>
  void rasterize_triangle(TVert2Frag tri[3], vec4f clip_pos[3],
                          const std::shared_ptr<TShader> &shader);
  template <class TVert2Frag>
  TVert2Frag interpolate_attr(TVert2Frag tri[3], const vec3f &weight);
  // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI ->
  // FRAGSHADER

  mat4f model = mat4f::Identity();
  mat4f view = mat4f::Identity();
  mat4f project = mat4f::Identity();

  // input buffer
  // std::map<int, std::vector<decltype(shader->arr2vert_cls)>> arr_bufs;
  // std::map<int, std::vector<int>> ind_bufs;

  // output buffer
  std::vector<uchar> encode_frame_buf;
  std::vector<vec4f> frame_buf;
  std::vector<float> depth_buf;
};

SoftRaster::SoftRaster(int w, int h) : width(w), height(h) {
  encode_frame_buf.resize(4 * w * h);
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

void SoftRaster::clear(int buffer_mask) {
  if (buffer_mask & SoftRaster::Buffer::Color) {
    std::fill(frame_buf.begin(), frame_buf.end(), vec4f::Zero());
  }
  if (buffer_mask & SoftRaster::Buffer::Depth) {
    std::fill(depth_buf.begin(), depth_buf.end(), 0.0f);
  }
}

template <class TArr2Vert, class TShader>
void SoftRaster::draw_arrays(const std::vector<TArr2Vert> &vertex_arr,
                             const std::shared_ptr<TShader> &shader,
                             SoftRaster::Primitive mode) {
  // printf("hello from drawArrays.\n");
  // draw triangles [0, pos_buf)
  // int size = 3;
  // int stride = 3;
  using TVert2Frag = decltype(shader->vert2frag_t());

  static_assert(std::is_base_of<IShader<TArr2Vert, TVert2Frag>, TShader>::value,
                "Shader dismatch TArr2Vert");

  auto mvp = project * view * model;
  shader->project = project;
  shader->view = view;
  shader->model = model;

  if (mode == Primitive::Triangle) {
    auto &curr_buf = vertex_arr;
    for (int i = 0; i < curr_buf.size(); i += 3) {
      TArr2Vert a2v[3] = {curr_buf[i], curr_buf[i + 1], curr_buf[i + 2]};
      TVert2Frag v2f[3];

      // printf(" draw with shader: %s::%s size of A2V, V2F: %d, %d\n",
      //        (shader->tag()).c_str(), __func__, (int)sizeof(TArr2Vert),
      //        (int)sizeof(shader->vert2frag_t()));

      // verts represent clip_pos
      vec4f verts[3] = {shader->vertex_shader(a2v[0], v2f[0]),
                        shader->vertex_shader(a2v[1], v2f[1]),
                        shader->vertex_shader(a2v[2], v2f[2])};

      // Homogeneous division
      for (auto &vert : verts) {
        vert.x() /= vert.w();
        vert.y() /= vert.w();
        vert.z() /= vert.w();
      }

      // Viewport transformation
      for (auto &vert : verts) {
        vert.x() = 0.5f * width * (vert.x() + 1.0f);
        vert.y() = 0.5f * height * (vert.y() + 1.0f);
      }
      // clip here

      // for (int j = 0; j < 3; j++) {
      //   v2f[j].screen_pos = verts[j];
      // }
      // scan triangles
      rasterize_triangle(v2f, verts, shader);
    }
  }
}

// void SoftRaster::draw_elements(SoftRaster::ind_buf_id ind_id,
//                                SoftRaster::Primitive mode) {}

int SoftRaster::get_next_id() { return next_id++; }

// void SoftRaster::set_shader(const std::shared_ptr<IShader> &_shader) {
//   shader = _shader;
// }

void SoftRaster::set_model(const mat4f &m) { model = m; }

void SoftRaster::set_view(const mat4f &v) { view = v; }

void SoftRaster::set_project(const mat4f &p) { project = p; }

void SoftRaster::set_pixel(const vec2i &coord, const vec4f &color) {
  frame_buf[width * coord.y() + coord.x()] = color;
}

int SoftRaster::get_index(int x, int y) { return width * y + x; }

// typename SoftRaster::arr_buf_id SoftRaster::load_array(
//     const std::vector<TApp2Vert> &vertex_arr) {
//   auto id = get_next_id();
//   arr_bufs.emplace(id, vertex_arr);
//   return {id};
// }

// typename SoftRaster::ind_buf_id SoftRaster::load_indices(
//     const std::vector<int> &inds) {
//   auto id = get_next_id();
//   ind_bufs.emplace(id, inds);
//   return {id};
// }

std::vector<uchar> &SoftRaster::encode_frame_buffer() {
  for (auto i = 0; i < width * height; i++) {
    encode_frame_buf[(i << 2) + 0] = (uchar)(255.0f * frame_buf[i].x());
    encode_frame_buf[(i << 2) + 1] = (uchar)(255.0f * frame_buf[i].y());
    encode_frame_buf[(i << 2) + 2] = (uchar)(255.0f * frame_buf[i].z());
    encode_frame_buf[(i << 2) + 3] = (uchar)(255.0f * frame_buf[i].w());
  }
  return encode_frame_buf;
}

template <class TVert2Frag, class TShader>
void SoftRaster::rasterize_triangle(TVert2Frag v2f[3], vec4f clip_pos[3],
                                    const std::shared_ptr<TShader> &shader) {
  // vec4f v[3] = {v2f[0].screen_pos, v2f[1].screen_pos, v2f[2].screen_pos};
  // vec4f view_pos[3] = {v2f[0].view_pos, v2f[1].view_pos, v2f[2].view_pos};

  using TArr2Vert = decltype(shader->arr2vert_t());
  static_assert(std::is_base_of<IShader<TArr2Vert, TVert2Frag>, TShader>::value,
                "Shader dismatch TVert2Frag");

  vec4f *v = clip_pos;
  vec2f bboxmin(v[0].x(), v[0].y());
  vec2f bboxmax(v[0].x(), v[0].y());
  vec2f _clamp(width - 1, height - 1);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      bboxmin[j] = std::max(0.0f, std::min(bboxmin[j], v[i][j]));
      bboxmax[j] = std::min(_clamp[j], std::max(bboxmax[j], v[i][j]));
    }
  }

  for (int x = (int)bboxmin.x(); x <= bboxmax.x(); x++) {
    for (int y = (int)bboxmin.y(); y <= bboxmax.y(); y++) {
      // bc means barycentric coords in screen space
      vec3f bc_screen = barycentric2D((float)x, (float)y, v);
      vec3f bc_clip(bc_screen.x() / v[0].w(), bc_screen.y() / v[1].w(),
                    bc_screen.z() / v[2].w());
      bc_clip = bc_clip / bc_clip.sum();

      float frag_depth = bc_clip.dot(vec3f(v[0].z(), v[1].z(), v[2].z()));
      if (bc_screen.x() < 0 || bc_screen.y() < 0 || bc_screen.z() < 0 ||
          depth_buf[get_index(x, y)] > frag_depth)
        continue;

      // decltype(shader->vert2frag_cls) payload;
      // payload.color << 0.5f, 1.0f, 0.0f, 1.0f;
      // printf("size of payload v2f: %d\n", sizeof(V2F));
      // vecNf a, b, c;
      // auto ptr_a = reinterpret_cast<float *>(v2f + 0);
      // auto ptr_b = reinterpret_cast<float *>(v2f + 1);
      // auto ptr_c = reinterpret_cast<float *>(v2f + 2);

      // const int arr_length = sizeof(v2f[0]) / sizeof(float);
      // Eigen::Matrix<float, arr_length, 1> attr_a =
      //     Eigen::Map<Eigen::Matrix<float, arr_length, 1>>(ptr_a);
      // Eigen::Matrix<float, arr_length, 1> attr_b =
      //     Eigen::Map<Eigen::Matrix<float, arr_length, 1>>(ptr_b);
      // Eigen::Matrix<float, arr_length, 1> attr_c =
      //     Eigen::Map<Eigen::Matrix<float, arr_length, 1>>(ptr_c);

      // Eigen::Matrix<float, arr_length, 3> attr_mat;
      // attr_mat << attr_a, attr_b, attr_c;

      auto result = interpolate_attr(v2f, bc_clip);

      // auto result = shader->interpolate_attr(v2f, bc_clip);
      auto color = shader->fragment_shader(result);
      set_pixel(vec2i(x, y), color);
      depth_buf[get_index(x, y)] = frag_depth;
    }
  }
}

template <class TVert2Frag>
TVert2Frag SoftRaster::interpolate_attr(TVert2Frag tri[3],
                                        const vec3f &weight) {
  const int arr_length = sizeof(tri[0]) / sizeof(float);
  auto attr_mat = Eigen::Map<Eigen::Matrix<float, arr_length, 3>>(
      reinterpret_cast<float *>(tri));

  return {attr_mat * weight};
}
}  // namespace mtr