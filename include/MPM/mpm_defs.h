#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <tuple>

using namespace Eigen;

namespace mpm {

struct GridAttr {
  float mass_i;
  Vector3f vel_in;
  Vector3f vel_i;
  Vector3f force_i;
  Vector3i Xi;
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
  int grid_size;
  int grid_w;
  int grid_h;
  int grid_l;

  // simulation factors
  float E = 50.0f;  // Young's modules
  float nu = 0.3f;  // Possion ratio

  float particle_density;
  float particle_mass;
  std::string model_path;
  Vector3f gravity;
  Vector3f world_area;
  float h;
};

Matrix3f neohookean_piola(float E, float nu, const Matrix3f& F);

}  // namespace mpm
