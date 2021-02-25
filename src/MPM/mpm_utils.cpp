#include "MPM/mpm_defs.h"

namespace mpm {

bool read_particles(const std::string& model_path,
                    std::vector<Vector3f>& positions) {
  std::ifstream input(model_path);
  std::string line;
  Vector3f pos;

  if (input) {
    positions.clear();
    while (std::getline(input, line)) {
      if (line[0] == 'v') {
        sscanf(line.c_str(), "v %f %f %f", &pos[0], &pos[1], &pos[2]);
      }
      positions.push_back(pos);
    }
    return true;
  } else {
    fprintf(stderr, "model_path:%s not found\n", model_path.c_str());
    return false;
  }
}

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
std::tuple<Vector3i, Matrix3f, Matrix3f> quatratic_interpolation(
    const Vector3f& particle_pos) {
  Vector3i base_node = floor(particle_pos.array() - 0.5f).cast<int>();
  Matrix3f wp, dwp;

  wp << calc_quadratic(base_node(0), particle_pos(0)),
      calc_quadratic(base_node(1), particle_pos(1)),
      calc_quadratic(base_node(2), particle_pos(2));

  dwp << calc_quadratic_grad(base_node(0), particle_pos(0)),
      calc_quadratic_grad(base_node(1), particle_pos(1)),
      calc_quadratic_grad(base_node(2), particle_pos(2));

  return {base_node, wp, dwp};
}
}  // namespace mpm