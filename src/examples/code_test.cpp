#include "MTR/math_defs.h"

int main() {
  struct Data {
    vec4f pos;
  } * tri;

  tri = new Data[3]{{{1, 0, 0, 1}}, {{0, 1, 0, 1}}, {{0, 0, 1, 1}}};

  vec3f weight{0.25, 0.31, 1 - 0.25 - 0.31};
  auto attr_mat =
      Eigen::Map<Eigen::Matrix<float, sizeof(Data) / sizeof(float), 3>>(
          reinterpret_cast<float*>(tri));
  auto result = attr_mat * weight;
  for (int i = 0; i < result.rows(); i++) {
    for (int j = 0; j < result.cols(); j++) {
      printf("%f%c", result(i, j), " \n"[j + 1 == result.cols()]);
    }
  }
  Data x{result};
  printf("Get result (%f, %f, %f, %f)\n", x.pos.x(), x.pos.y(), x.pos.z(),
         x.pos.z());
  return 0;
}