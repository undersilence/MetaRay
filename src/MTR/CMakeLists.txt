
project(MetaCore)

add_library(MetaCore STATIC math_defs.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(MetaCore PRIVATE Eigen3::Eigen)