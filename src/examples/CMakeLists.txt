project(MetaRayExamples)
add_executable(triangle_test triangle_test.cpp)

find_package(Eigen3 CONFIG REQUIRED) 
find_package(glfw3 CONFIG REQUIRED)

target_link_libraries(triangle_test PRIVATE Eigen3::Eigen)
target_link_libraries(triangle_test PRIVATE MetaCore)

add_executable(rotate_box rotate_box.cpp)
target_link_libraries(rotate_box PRIVATE glad)
target_link_libraries(rotate_box PRIVATE Eigen3::Eigen)
target_link_libraries(rotate_box PRIVATE glfw)
target_link_libraries(rotate_box PRIVATE MetaCore)