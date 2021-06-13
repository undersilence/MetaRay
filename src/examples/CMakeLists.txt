project(MetaRayExamples)
add_executable(triangle_test triangle_test.cpp)

find_package(glfw3 CONFIG REQUIRED)

target_link_libraries(triangle_test PRIVATE MetaCore)

add_executable(rotate_box rotate_box.cpp)
target_link_libraries(rotate_box PRIVATE glad)
target_link_libraries(rotate_box PRIVATE MetaCore)

if(glfw3_FOUND) 
    target_link_libraries(rotate_box PRIVATE glfw)
endif()