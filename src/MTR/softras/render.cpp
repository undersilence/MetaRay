#include "MTR/softras/render.h"

static vec3f barycentric2D(float x, float y, vec4f *v) {
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

template <class TApp2Vert, class TVert2Frag>
SoftRaster<TApp2Vert, TVert2Frag>::SoftRaster(int w, int h)
    : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::clear(SoftRaster::Buffer buffer_mask) {
  if (buffer_mask & SoftRaster::Buffer::Color) {
    std::fill(frame_buf.begin(), frame_buf.end(), vec4f::Zero());
  }
  if (buffer_mask & SoftRaster::Buffer::Depth) {
    std::fill(depth_buf.begin(), depth_buf.end(), 0.0f);
  }
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::draw_arrays(
    SoftRaster<TApp2Vert, TVert2Frag>::arr_buf_id arr_id,
    SoftRaster<TApp2Vert, TVert2Frag>::Primitive mode) {
  if (!shader) {
    // add some log here
    return;
  }
  printf("hello from drawArrays.\n");
  // draw triangles [0, pos_buf)
  // int size = 3;
  // int stride = 3;

  auto mvp = project * view * model;
  shader->project = project;
  shader->view = view;
  shader->model = model;

  if (mode == Primitive::Triangle) {
    auto &curr_buf = arr_bufs[arr_id.arr_id];
    for (int i = 0; i < curr_buf.size(); i += 3) {
      TApp2Vert a2v[3] = {curr_buf[i], curr_buf[i + 1], curr_buf[i + 2]};
      TVert2Frag v2f[3];
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
      rasterize_triangle(v2f, verts);
    }
  }
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::draw_elements(
    SoftRaster::ind_buf_id ind_id, SoftRaster::Primitive mode) {}

template <class TApp2Vert, class TVert2Frag>
int SoftRaster<TApp2Vert, TVert2Frag>::get_next_id() {
  return next_id++;
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::set_shader(
    const std::shared_ptr<IShader<TApp2Vert, TVert2Frag>> &_shader) {
  shader = _shader;
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::set_model(const mat4f &m) {
  model = m;
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::set_view(const mat4f &v) {
  view = v;
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::set_project(const mat4f &p) {
  project = p;
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::set_pixel(const vec2i &coord,
                                                  const vec4f &color) {
  frame_buf[width * coord.y() + coord.x()] = color;
}

template <class TApp2Vert, class TVert2Frag>
int SoftRaster<TApp2Vert, TVert2Frag>::get_index(int x, int y) {
  return width * y + x;
}

template <class TApp2Vert, class TVert2Frag>
typename SoftRaster<TApp2Vert, TVert2Frag>::arr_buf_id
SoftRaster<TApp2Vert, TVert2Frag>::load_array(
    const std::vector<TApp2Vert> &vertex_arr) {
  auto id = get_next_id();
  arr_bufs.emplace(id, vertex_arr);
  return {id};
}

template <class TApp2Vert, class TVert2Frag>
typename SoftRaster<TApp2Vert, TVert2Frag>::ind_buf_id
SoftRaster<TApp2Vert, TVert2Frag>::load_indices(const std::vector<int> &inds) {
  auto id = get_next_id();
  ind_bufs.emplace(id, inds);
  return {id};
}

template <class TApp2Vert, class TVert2Frag>
void SoftRaster<TApp2Vert, TVert2Frag>::rasterize_triangle(TVert2Frag v2f[3],
                                                           vec4f clip_pos[3]) {
  // vec4f v[3] = {v2f[0].screen_pos, v2f[1].screen_pos, v2f[2].screen_pos};
  // vec4f view_pos[3] = {v2f[0].view_pos, v2f[1].view_pos, v2f[2].view_pos};
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
      auto color =
          shader->fragment_shader(shader->interpolate_attr(v2f, bc_clip));
      frame_buf[get_index(x, y)] = color;
      depth_buf[get_index(x, y)] = frag_depth;
    }
  }
}

template class SoftRaster<A2V, V2F>;