#include "MPM/mpm_defs.h"

using namespace Eigen;
namespace mpm {

// neohookean model
class MPMSim {
 public:
  MPMSim(SimInfo info);

  void grid_initialize();
  void particle_initialize();
  void substep();

 protected:
  SimInfo sim_info;
  Particle* particles;
  GridAttr* grid_attrs;

  void P2G();
  void update_grid_force();
  void update_grid_velocity();
  void update_F();
  void G2P();

  // handle collision
  void paritcle_collision();
  void grid_collision();
};
}  // namespace mpm