project(MaterialPointMethod)


add_executable(MPM main.cpp mpm_sim.cpp mpm_utils.cpp mpm_defs.cpp)


find_package(Eigen3 CONFIG REQUIRED) 
target_link_libraries(MPM PRIVATE Eigen3::Eigen)