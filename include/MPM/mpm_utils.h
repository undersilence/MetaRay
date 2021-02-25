#pragma once
#include "MPM/mpm_defs.h"

namespace mpm {

bool read_particles(const std::string& model_path,
                    std::vector<Vector3f>& positions);

inline Vector3f calc_quadratic(float o, float x);

inline Vector3f calc_quadratic_grad(float o, float x);
// under gridspace coords
std::tuple<Vector3i, Matrix3f, Matrix3f> quatratic_interpolation(
    const Vector3f& particle_pos);
}  // namespace mpm