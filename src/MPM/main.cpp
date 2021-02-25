#include <iostream>
#include <memory>

#include "MPM/mpm_sim.h"
using namespace std;
using namespace Eigen;

int main() {
  auto sim = make_shared<mpm::MPMSim>();
  sim->mpm_demo();
  printf("hello from mpm!\n");

  return 0;
}