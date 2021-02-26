#include "MPM/mpm_sim.h"

#include "MPM/mpm_utils.h"

namespace mpm {

void MPMSim::mpm_demo() {
  // initialize logger
  mpm::MPMLog::init();

  float density = 1.0f;
  float mass = 1.0f;
  Vector3f gravity{0.0f, -9.8f, 0.0f};
  Vector3f area{1.0f, 1.0f, 1.0f};
  float h = 0.2f;

  mpm_initialize(density, mass, "../models/small_cube.obj", gravity, area, h);

  int frame_rate = 60;
  float dt = 1.0f / frame_rate;
  int frame_count = 300;

  for (int curr_frame = 1; curr_frame <= frame_count; curr_frame++) {
    MPM_INFO("now processing frame: {}", curr_frame);
    substep(dt);
    export_result("../output/", curr_frame);
  }
}

void MPMSim::substep(float dt) {
  // TODO: add profiler later
  prestep();
  transfer_P2G();
  add_gravity();
  update_grid_force(neohookean_piola);
  update_grid_velocity(dt);
  solve_grid_boundary();
  update_F(dt);
  transfer_G2P();
  advection(dt);
}

bool MPMSim::export_result(const std::string& export_dir, int curr_frame) {
  std::vector<Vector3f> positions(sim_info.particle_size);
  for (int i = 0; i < sim_info.particle_size; i++) {
    positions[i] = particles[i].pos_p;
  }
  auto export_path = export_dir + std::to_string(curr_frame) + ".bgeo";
  return write_particles(export_path, positions);
}

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

void MPMSim::prestep() {
  for (int i = 0; i < sim_info.grid_size; i++) {
    grid_attrs[i].mass_i = 0;
    grid_attrs[i].force_i = Vector3f::Zero();
    grid_attrs[i].vel_i = Vector3f::Zero();
    grid_attrs[i].vel_in = Vector3f::Zero();
  }
}

void MPMSim::transfer_P2G() {
  active_nodes.clear();
  // Matrix3f wp, dwp;
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    // convert particles position to grid space by divide h
    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / sim_info.h);

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          // note: do not use auto here (cause error in release mode)
          Vector3i curr_node = base_node + Vector3i(i, j, k);
          int index = curr_node(0) * sim_info.grid_h * sim_info.grid_l +
                      curr_node(1) * sim_info.grid_l + curr_node(2);

          // check if particles run out of boundaries
          MPM_ASSERT(0 <= index && index < sim_info.grid_size);
          // MPM_INFO("curr_node:\n{}", curr_node);

          float wijk = wp(0, i) * wp(1, j) * wp(2, k);

          grid_attrs[index].mass_i += wijk * particles[iter].mass_p;
          // accumulate momentum at time n
          grid_attrs[index].vel_in +=
              wijk * particles[iter].mass_p * particles[iter].vel_p;
        }
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

void MPMSim::add_gravity() {
  MPM_ASSERT(active_nodes.size() < sim_info.grid_size);
  for (auto iter = 0; iter < active_nodes.size(); iter++) {
    int index = active_nodes[iter];
    grid_attrs[index].force_i += sim_info.gravity * grid_attrs[index].mass_i;
  }
}

void MPMSim::update_grid_force(
    std::function<Matrix3f(float, float, const Matrix3f&)> constitutive_model) {
  // update grid forcing from particles F(deformation gradients)

  MPM_INFO("{} start", __func__);
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    auto F = particles[iter].F;
    auto vol_p = particles[iter].volume_p;
    auto h = sim_info.h;

    // use constitutive_model
    Matrix3f piola = constitutive_model(sim_info.E, sim_info.nu, F);

    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / h);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          Vector3i curr_node = base_node + Vector3i(i, j, k);
          Vector3f grad_wip{dwp(0, i) * wp(1, j) * wp(2, k) / h,
                            wp(0, i) * dwp(1, j) * wp(2, k) / h,
                            wp(0, i) * wp(1, j) * dwp(2, k) / h};

          auto index = curr_node.x() * sim_info.grid_h * sim_info.grid_l +
                       curr_node.y() * sim_info.grid_l + curr_node.z();
          MPM_ASSERT(0 <= index && index < sim_info.grid_size);

          grid_attrs[index].force_i -=
              vol_p * (piola * F.transpose()) * grad_wip;
        }
  }

  MPM_INFO("{} end", __func__);
}

void MPMSim::update_grid_velocity(float dt) {
  for (auto i = 0; i < active_nodes.size(); i++) {
    int index = active_nodes[i];
    // vel_n+1 = vel_n + f_i / m_i * dt
    grid_attrs[index].vel_i =
        grid_attrs[index].vel_in +
        grid_attrs[index].force_i / grid_attrs[index].mass_i * dt;
  }
}

void MPMSim::update_F(float dt) {
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    auto F = particles[iter].F;
    auto h = sim_info.h;
    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / h);

    Matrix3f weight = Matrix3f::Zero();
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          Vector3i curr_node = base_node + Vector3i(i, j, k);
          Vector3f grad_wip{dwp(0, i) * wp(1, j) * wp(2, k) / h,
                            wp(0, i) * dwp(1, j) * wp(2, k) / h,
                            wp(0, i) * wp(1, j) * dwp(2, k) / h};

          auto index = curr_node.x() * sim_info.grid_h * sim_info.grid_l +
                       curr_node.y() * sim_info.grid_l + curr_node.z();

          weight += grid_attrs[index].vel_in * grad_wip.transpose();
        }
    weight *= dt;
    weight += Matrix3f::Identity();
    particles[iter].F = weight * F;
  }
}

void MPMSim::transfer_G2P() {
  for (int iter = 0; iter < sim_info.particle_size; iter++) {
    auto [base_node, wp, dwp] =
        quatratic_interpolation(particles[iter].pos_p / sim_info.h);
    Vector3f v_pic = Vector3f::Zero();
    Vector3f v_flip = particles[iter].vel_p;

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        for (int k = 0; k < 3; k++) {
          Vector3i curr_node = base_node + Vector3i(i, j, k);
          auto wijk = wp(0, i) * wp(1, j) * wp(2, k);
          auto index = curr_node.x() * sim_info.grid_h * sim_info.grid_l +
                       curr_node.y() * sim_info.grid_l + curr_node.z();
          v_pic += wijk * grid_attrs[index].vel_i;
          v_flip += wijk * (grid_attrs[index].vel_i - grid_attrs[index].vel_in);
        }

    particles[iter].vel_p =
        (1 - sim_info.alpha) * v_pic + sim_info.alpha * v_flip;
  }
}

void MPMSim::advection(float dt) {
  for (int i = 0; i < sim_info.particle_size; i++) {
    particles[i].pos_p += dt * particles[i].vel_p;
    if (i < 5)
      MPM_INFO("particle_{} vel:({},{},{}), move to ({},{},{})", i,
               particles[i].vel_p[0], particles[i].vel_p[1],
               particles[i].vel_p[2], particles[i].pos_p[0],
               particles[i].pos_p[1], particles[i].pos_p[2]);
  }
}

void MPMSim::solve_grid_boundary(int thickness) {
  // Sticky boundary
  auto [W, H, L] = std::tie(sim_info.grid_w, sim_info.grid_h, sim_info.grid_l);
  // check x-axis bound
  for (int i = 0; i < thickness; i++) {
    for (int j = 0; j < H; j++) {
      for (int k = 0; k < L; k++) {
        int index1 = i * H * L + j * L + k;
        int index2 = (W - i - 1) * H * L + j * L + k;
        if (grid_attrs[index1].vel_i[0] < 0) {
          grid_attrs[index1].vel_i[0] = 0.0f;
        }
        if (grid_attrs[index2].vel_i[0] > 0) {
          grid_attrs[index2].vel_i[0] = 0.0f;
        }
      }
    }
  }
  // check y-axis bound
  for (int i = 0; i < W; i++) {
    for (int j = 0; j < thickness; j++) {
      for (int k = 0; k < L; k++) {
        int index1 = i * H * L + j * L + k;
        int index2 = i * H * L + (H - j - 1) * L + k;
        if (grid_attrs[index1].vel_i[1] < 0) {
          grid_attrs[index1].vel_i[1] = 0.0f;
        }
        if (grid_attrs[index2].vel_i[1] > 0) {
          grid_attrs[index2].vel_i[1] = 0.0f;
        }
      }
    }
  }
  // check z-axis bound
  for (int i = 0; i < W; i++) {
    for (int j = 0; j < H; j++) {
      for (int k = 0; k < thickness; k++) {
        int index1 = i * H * L + j * L + k;
        int index2 = (W - i - 1) * H * L + j * L + (L - k - 1);
        if (grid_attrs[index1].vel_i[2] < 0) {
          grid_attrs[index1].vel_i[2] = 0.0f;
        }
        if (grid_attrs[index2].vel_i[2] > 0) {
          grid_attrs[index2].vel_i[2] = 0.0f;
        }
      }
    }
  }
}

}  // namespace mpm