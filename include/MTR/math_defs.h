#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>

typedef double scalar;
typedef unsigned long long uint64;

typedef Eigen::Matrix<scalar, 2, 1> vec2s;
typedef Eigen::Matrix<float, 2, 1> vec2f;
typedef Eigen::Matrix<int, 2, 1> vec2i;
typedef Eigen::Matrix<scalar, 3, 1> vec3s;
typedef Eigen::Matrix<float, 3, 1> vec3f;
typedef Eigen::Matrix<int, 3, 1> vec3i;
typedef Eigen::Matrix<scalar, 4, 1> vec4s;
typedef Eigen::Matrix<float, 4, 1> vec4f;
typedef Eigen::Matrix<int, 4, 1> vec4i;

typedef Eigen::Matrix<scalar, 4, 4> mat4s;
typedef Eigen::Matrix<float, 4, 4> mat4f;
typedef Eigen::Matrix<int, 4, 4> mat4i;

