#include <iostream>

#include "MPM/mpm_sim.h"
using namespace std;
using namespace Eigen;

int main() {
  mpm::SimInfo info;
  mpm::MPMSim sim(info);

  printf("hello from mpm!\n");

  return 0;
}