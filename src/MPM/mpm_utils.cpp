#include "MPM/mpm_defs.h"

namespace mpm {

inline Vector3f calc_quadratic(float o, float x) {
  // +-(o)------(o+1)--(x)--(o+2)-+
  float d0 = x - o;
  float d1 = d0 - 1;
  float d2 = 1 - d1;

  return {0.5f * (1.5f - d0) * (1.5f - d0), 0.75f - d1 * d1,
          0.5f * (1.5f - d2) * (1.5f - d2)};
}

inline Vector3f calc_quadratic_grad(float o, float x) {
  float d0 = x - o;
  float d1 = d0 - 1;
  float d2 = 1 - d1;

  return {d0 - 1.5f, -2 * d1, 1.5f - d2};
}

// under gridspace coords
std::tuple<Matrix3f, Matrix3f> quatratic_interpolation(
    Vector3i& base_node, const Vector3f& particle_pos) {
  base_node = floor(particle_pos.array() - 0.5f).cast<int>();
  Matrix3f wp, dwp;

  wp << calc_quadratic(base_node(0), particle_pos(0)),
      calc_quadratic(base_node(1), particle_pos(1)),
      calc_quadratic(base_node(2), particle_pos(2));

  dwp << calc_quadratic_grad(base_node(0), particle_pos(0)),
      calc_quadratic_grad(base_node(1), particle_pos(1)),
      calc_quadratic_grad(base_node(2), particle_pos(2));

  return {wp, dwp};
}
}  // namespace mpm