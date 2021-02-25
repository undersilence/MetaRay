#include "MPM/mpm_sim.h"

#include "MPM/mpm_utils.h"

namespace mpm {

void MPMSim::mpm_demo() {}

bool MPMSim::mpm_initialize(float particle_density, float particle_mass,
                            const std::string& model_path,
                            const Vector3f& gravity, const Vector3f& world_area,
                            float h) {
  std::vector<Vector3f> positions;
  if (read_particles(model_path, positions)) {
    sim_info.particle_density = particle_density;
    sim_info.particle_mass = particle_mass;
    sim_info.model_path = model_path;
    sim_info.gravity = gravity;
    sim_info.world_area = world_area;
    sim_info.h = h;

    // particles_initialize
    sim_info.particle_size = positions.size();
    particles.resize(positions.size());
    for (int i = 0; i < sim_info.particle_size; i++) {
      particles[i].pos_p = positions[i];
      particles[i].mass_p = particle_mass;
      particles[i].volume_p = particle_mass / particle_density;
      particles[i].vel_p = Vector3f::Zero();
      particles[i].F = Matrix3f::Identity();
    }

    // grid_initialize
    auto [W, H, L] = std::make_tuple<int, int, int>(
        world_area.x() / h + 1, world_area.y() / h + 1, world_area.z() / h + 1);

    sim_info.grid_w = W;
    sim_info.grid_h = H;
    sim_info.grid_l = L;
    sim_info.grid_size = W * H * L;
    grid_attrs.resize(sim_info.grid_size);
    for (int i = 0; i < W; i++)
      for (int j = 0; j < H; j++)
        for (int k = 0; k < L; k++) {
          int index = i * H * L + j * L + k;
          grid_attrs[index].mass_i = 0;
          grid_attrs[index].force_i = Vector3f::Zero();
          grid_attrs[index].vel_i = Vector3f::Zero();
          grid_attrs[index].vel_in = Vector3f::Zero();
          grid_attrs[index].Xi = Vector3i(i, j, k);
        }

    return true;
  } else {
    fprintf(stderr, "[MPM]:model data not found! %s\n",
            sim_info.model_path.c_str());
    return false;
  }
}

void MPMSim::transfer_P2G() {
  // Matrix3f wp, dwp;
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    // convert particles position to grid space by divide h
    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / sim_info.h);

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          auto curr_node = base_node + Vector3i(i, j, k);
          int index = curr_node(0) * sim_info.grid_h * sim_info.grid_l +
                      curr_node(1) * sim_info.grid_l + curr_node(2);
          float wijk = wp(0, i) * wp(1, j) * wp(2, k);

          grid_attrs[index].mass_i += wijk * particles[iter].mass_p;
          // accumulate momentum at time n
          grid_attrs[index].vel_in +=
              wijk * particles[iter].mass_p * particles[iter].vel_p;
        }

    for (int iter = 0; iter < sim_info.grid_size; iter++) {
      if (grid_attrs[iter].mass_i > 1e-15) {
        active_nodes.push_back(iter);
        grid_attrs[iter].vel_in =
            grid_attrs[iter].vel_in / grid_attrs[iter].mass_i;
      } else {
        grid_attrs[iter].vel_in = Vector3f::Zero();
      }
    }
  }
}

void MPMSim::add_gravity() {
  for (int iter = 0; iter < active_nodes.size(); iter++) {
    int index = active_nodes[iter];
    grid_attrs[index].force_i += sim_info.gravity * grid_attrs[index].mass_i;
  }
}

void MPMSim::update_grid_force(
    std::function<Matrix3f(const Matrix3f&)> constitutive_model) {
  // update grid forcing from particles F(deformation gradients)
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    auto F = particles[iter].F;
    auto vol_p = particles[iter].volume_p;
    auto h = sim_info.h;

    // use constitutive_model
    Matrix3f piola = constitutive_model(F);

    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / h);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          auto curr_node = base_node + Vector3i(i, j, k);
          Vector3f grad_wip{dwp(0, i) * wp(1, j) * wp(2, k) / h,
                            wp(0, i) * dwp(1, j) * wp(2, k) / h,
                            wp(0, i) * wp(1, j) * dwp(2, k) / h};

          auto index = curr_node.x() * sim_info.grid_h * sim_info.grid_l +
                       curr_node.y() * sim_info.grid_l + curr_node.z();

          grid_attrs[index].force_i -=
              vol_p * (piola * F.transpose()) * grad_wip;
        }
  }
}

void update_grid_velocity(float dt) {}

}  // namespace mpm