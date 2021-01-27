#include "MTR/softras/render.h"

static vec3f barycentric2D(float x, float y, vec4f *v) {
  float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
             (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
  float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
             (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
  float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
             (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
  return {c1, c2, c3};
}

SoftRaster::SoftRaster(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

SoftRaster::clear(unsigned int buffer_mask) {}

void SoftRaster::draw_arrays(SoftRaster::Primitive mode) {
  printf("hello from drawArrays.\n");
  // draw triangles [0, pos_buf)
  int size = 3;
  int stride = 3;

  auto mvp = project * view * model;
  shader->project = project;
  shader->view = view;
  shader->model = model;

  if (mode == Primitive::Triangle) {
    auto &curr_buf = arr_bufs[0];
    for (int i = size; i < curr_buf.size(); i += 3 * stride) {
      A2V a2v[3];
      V2F v2f[3];

      // verts represent clip_pos
      vec4f verts[3];
      // only configure positions for test

      for (int j = 0; j < 3; j++) {
        a2v[j].position << curr_buf[i - 3 + j * size], curr_buf[i - 2 + j * size], curr_buf[i - 1 + j * size];
        // a2v[j].project = project;
        // a2v[j].model = model;
        // a2v[j].view = view;

        verts[j] = shader->vertex_shader(a2v[j], v2f[j]);
      }

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

      for (int j = 0; j < 3; j++) {
        v2f[j].screen_pos = verts[i];
      }
      // scan triangles
      rasterize_triangle(v2f);
    }
  }
}

void SoftRaster::draw_elements() {}

int SoftRaster::get_next_id() { return next_id++; }

void SoftRaster::set_model(const mat4f &m) { model = m; }

void SoftRaster::set_view(const mat4f &v) { view = v; }

void SoftRaster::set_project(const mat4f &p) { project = p; }

void SoftRaster::set_pixel(const vec2i &coord, const vec4f &color) { frame_buf[width * coord.y() + coord.x()] = color; }

int SoftRaster::get_index(int x, int y) { return width * y + x; }

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

void SoftRaster::rasterize_triangle(V2F v2f[3]) {

  vec4f v[3] = {v2f[0].screen_pos, v2f[1].screen_pos, v2f[2].screen_pos};
  // vec4f view_pos[3] = {v2f[0].view_pos, v2f[1].view_pos, v2f[2].view_pos};
  vec4f color[3] = {v2f[0].color, v2f[1].color, v2f[2].color};

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
      vec3f bc_clip(bc_screen.x() / v[0].w(), bc_screen.y() / v[1].w(), bc_screen.z() / v[2].w());
      bc_clip = bc_clip / bc_clip.sum();

      float frag_depth = bc_clip.dot(vec3f(v[0].z(), v[1].z(), v[2].z()));
      if (bc_screen.x() < 0 || bc_screen.y() < 0 || bc_screen.z() < 0 || depth_buf[get_index(x, y)] > frag_depth)
        continue;

      V2F payload;
      payload.color << 0.5f, 1.0f, 0.0f, 1.0f;
      // payload.color << bc_clip.dot(
      //     vec3f(color[0].x(), color[1].x(), color[2].x())),
      //     bc_clip.dot(vec3f(color[0].y(), color[1].y(), color[2].y())),
      //     bc_clip.dot(vec3f(color[0].z(), color[1].z(), color[2].z())),
      //     bc_clip.dot(vec3f(color[0].w(), color[1].w(), color[2].w()));

      frame_buf[get_index(x, y)] = shader->fragment_shader(payload);
      depth_buf[get_index(x, y)] = frag_depth;
    }
  }
}
