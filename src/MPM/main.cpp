
#include <iostream>
#include <memory>

#include "MPM/mpm_sim.h"
#include "MPM/mpm_utils.h"
using namespace std;
using namespace Eigen;
using namespace mpm;

void quatratic_test() {
  MPM_PROFILE_FUNCTION();
  MPM_INFO("{} start", __func__);
  float h = 0.02f;
  int W = 1.0f / h + 1;
  int H = 1.0f / h + 1;
  int L = 1.0f / h + 1;
  Vector3f pos(0.648932, 0.121521, 0.265484);
  auto [base_node, wp, dwp] = mpm::quatratic_interpolation(pos / h);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++) {
        float wi = wp(i, 0), wj = wp(j, 1), wk = wp(k, 2);
        float dwi = dwp(i, 0), dwj = dwp(j, 1), dwk = dwp(k, 2);
        float wijk = wi * wj * wk;
        Vector3i curr_node = base_node + Vector3i(i, j, k);

        int index = curr_node(0) * H * L + curr_node(1) * L + curr_node(2);
        Vector3f grad_wp(dwi * wj * wk / h, wi * dwj * wk / h,
                         wi * wj * dwk / h);

        MPM_INFO("offset: {}", Vector3i(i, j, k).transpose());
        MPM_INFO("weight_ijk: {}", wijk);
        MPM_INFO("grad_wp: {}", grad_wp.transpose());
      }
  MPM_INFO("{} end", __func__);
}

int main() {
  // initialize logger
  mpm::MPMLog::init();
  // quatratic_test();
  auto sim = make_shared<mpm::MPMSim>();
  sim->mpm_demo();

  printf("mpm finished!\n");
  return 0;
}