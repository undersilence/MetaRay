#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <tuple>

#include "spdlog/fmt/ostr.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

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
  float E = 50.0f;      // Young's modules
  float nu = 0.3f;      // Possion ratio
  float alpha = 0.95f;  // 0.95 flip/pic

  float particle_density;
  float particle_mass;
  std::string model_path;
  Vector3f gravity;
  Vector3f world_area;
  float h;
};

Matrix3f neohookean_piola(float E, float nu, const Matrix3f& F);

// define logger
class MPMLog {
 public:
  static void init();
  MPMLog() = default;
  virtual ~MPMLog() = default;

  inline static const std::shared_ptr<spdlog::logger>& get_logger() {
    return s_logger;
  }

 private:
  static std::shared_ptr<spdlog::logger> s_logger;
};

class MPMProfiler {
 public:
  MPMProfiler(const std::string& tag);
  virtual ~MPMProfiler();

 private:
  std::string tag;
  std::chrono::high_resolution_clock::time_point start;
};

#ifndef DISTRIBUTE
// Client log macros
#define MPM_FATAL(...) MPMLog::get_logger()->fatal(__VA_ARGS__)
#define MPM_ERROR(...) MPMLog::get_logger()->error(__VA_ARGS__)
#define MPM_WARN(...) MPMLog::get_logger()->warn(__VA_ARGS__)
#define MPM_INFO(...) MPMLog::get_logger()->info(__VA_ARGS__)
#define MPM_TRACE(...) MPMLog::get_logger()->trace(__VA_ARGS__)

#define MPM_ASSERT(...) assert(__VA_ARGS__)

#define MPM_FUNCTION_SIG __func__
#define MPM_PROFILE(tag) MPMProfiler timer##__LINE__(tag)
#define MPM_PROFILE_FUNCTION() MPM_PROFILE(MPM_FUNCTION_SIG)

#else

#define MPM_FATAL(...)
#define MPM_ERROR(...)
#define MPM_WARN(...)
#define MPM_INFO(...)
#define MPM_TRACE(...)

#define MPM_ASSERT(...)

#define MPM_FUNCTION_SIG
#define MPM_PROFILE(tag)
#define MPM_PROFILE_FUNCTION()

#endif

}  // namespace mpm
