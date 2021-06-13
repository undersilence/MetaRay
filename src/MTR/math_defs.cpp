#include "MTR/math_defs.hpp"

float clamp(float min, float max, float a) {
  return a > min ? (a < max ? a : max) : min;
}

float to_radians(float degree) { return degree / 180.0f * MTR_PI; }
float to_degrees(float radian) { return radian / MTR_PI * 180.0f; }

// define reversed-z perspective projection
mat4f perspective(float fovy, float aspect, float near, float far) {
  const float tan_half_fovy = std::tan(fovy * 0.5f);

  mat4f result = mat4f::Zero();
  result(0, 0) = 1.0f / (aspect * tan_half_fovy);
  result(1, 1) = 1.0f / (tan_half_fovy);
  result(2, 2) = -near / (far - near);
  result(3, 2) = 1.0f;
  result(2, 3) = (near * far) / (far - near);
  return result;
}

mat4f ortho(float left, float right, float bottom, float top, float near,
            float far) {
  return {};
}

mat4f look_at(const vec3f &eye, const vec3f &center, const vec3f &up) {
  return {};
}

mat4f translate_mat(float offset_x, float offset_y, float offset_z) {
  mat4f result = mat4f::Identity();
  result(0, 3) = offset_x;
  result(1, 3) = offset_y;
  result(2, 3) = offset_z;
  return result;
}

mat4f rotate_mat(const vec3f &pivot, float angle) {
  float u = sin(angle), v = cos(angle);
  return mat4f::Identity();
}

mat4f scale_mat(float s_x, float s_y, float s_z) {
  mat4f result = mat4f::Identity();
  result(0, 0) = s_x;
  result(1, 1) = s_y;
  result(2, 2) = s_z;
  return result;
}

vec3f barycentric2D(float x, float y, vec4f *v) {
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