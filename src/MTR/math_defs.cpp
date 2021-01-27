#include "MTR/math_defs.h"

float to_radius(float degree) { return degree / 180.0f * MTR_PI; }
float to_degree(float radius) { return radius / MTR_PI * 180.0f; }

// define reversed-z perspective projection
mat4f perspective(float fovy, float aspect, float near, float far) {
  const float tan_half_fovy = tanf(to_radius(fovy * 0.5f));

  mat4f result = mat4f::Zero();
  result(0, 0) = 1.0f / (aspect * tan_half_fovy);
  result(1, 1) = 1.0f / (tan_half_fovy);
  result(2, 2) = -near / (far - near);
  result(2, 3) = (near * far) / (far - near);
  result(3, 2) = 1.0f;
  return result;
}

mat4f ortho(float left, float right, float bottom, float top, float near, float far) { return {}; }

mat4f look_at(const vec3f &eye, const vec3f &center, const vec3f &up) { return {}; }
