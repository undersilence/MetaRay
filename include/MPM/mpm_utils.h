#include "MPM/mpm_defs.h"

namespace mpm {
inline Vector3f calc_quadratic(float o, float x);

inline Vector3f calc_quadratic_grad(float o, float x);
// under gridspace coords
std::tuple<Matrix3f, Matrix3f> quatratic_interpolation(
    Vector3i& base_node, const Vector3f& particle_pos);
}  // namespace mpm