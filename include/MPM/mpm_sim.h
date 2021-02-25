#pragma once
#include "MPM/mpm_defs.h"

using namespace Eigen;
namespace mpm {

// neohookean model
class MPMSim {
 public:
  MPMSim() = default;
  virtual ~MPMSim() = default;

  void mpm_demo();

  bool mpm_initialize(float particle_density, float particle_mass,
                      const std::string& model_path, const Vector3f& gravity,
                      const Vector3f& world_area, float h);
  // void grid_initialize();
  // void particle_initialize();
  void substep(float dt);

 protected:
  SimInfo sim_info;
  std::vector<Particle> particles;
  std::vector<GridAttr> grid_attrs;

  // storage the degrees of freedoms
  std::vector<int> active_nodes;

  void transfer_P2G();
  void add_gravity();
  // TODO: support variety
  void update_grid_force(
      std::function<Matrix3f(const Matrix3f&)> constitutive_model);
  void update_grid_velocity(float dt);
  void update_F();
  void transfer_G2P();

  // handle collision
  void paritcle_collision();
  void grid_collision();
};
}  // namespace mpm