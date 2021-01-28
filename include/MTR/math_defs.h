#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/StdVector>

const float MTR_PI = 3.1415926535f;

typedef double scalar;
typedef unsigned long long uint64;
typedef unsigned char uchar;

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

float clamp(float min, float max, float a);

float to_degrees(float radius);
float to_radians(float degree);

mat4f perspective(float fovy, float aspect, float near, float far);
mat4f ortho(float left, float right, float bottom, float top, float near,
            float far);
mat4f look_at(const vec3f &eye, const vec3f &center, const vec3f &up);

mat4f translate_mat(float offset_x, float offset_y, float offset_z);
mat4f rotate_mat(const vec3f &pivot, float angle);
mat4f scale_mat(float s_x, float s_y, float s_z);

vec3f barycentric2D(float x, float y, vec4f *v);
