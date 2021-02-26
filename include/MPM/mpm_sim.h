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
  bool export_result(const std::string& export_path, int curr_frame);

 private:
  SimInfo sim_info;
  std::vector<Particle> particles;
  std::vector<GridAttr> grid_attrs;

  // storage the degrees of freedoms
  std::vector<int> active_nodes;

  void prestep();
  void transfer_P2G();
  void add_gravity();
  // TODO: support variety
  //  E : float, nu : float, F : Matrix3f
  void update_grid_force(std::function<Matrix3f(float, float, const Matrix3f&)>
                             constitutive_model);
  void update_grid_velocity(float dt);
  void update_F(float dt);
  void transfer_G2P();
  void advection(float dt);

  // handle collision
  // void solve_paritcle_collision();
  // void solve_grid_collision();
  void solve_grid_boundary(int thickness = 2);
};
}  // namespace mpm