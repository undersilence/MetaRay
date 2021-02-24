#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>
#include <cmath>
#include <iostream>

using namespace Eigen;

namespace mpm {

struct GridAttr {
  float mass_i;
  Vector3f vel_in;
  Vector3f vel_i;
  Vector3f force_i;
  Vector3i X_i;
};

struct Particle {
  float mass_p;
  float volume_p;
  Vector3f pos_p;
  Vector3f vel_p;
  Matrix3f F;  // deformation gradients
};

struct SimInfo {
  int particle_size;
  float h;
  int grid_size;
  int grid_w;
  int grid_h;
  int grid_l;
};

}  // namespace mpm
