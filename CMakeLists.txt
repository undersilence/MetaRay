cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

project(MetaRay)

include_directories(include)
include_directories(external/glad/include)
include_directories(external/partio/src/lib)
include_directories(external/spdlog/include)

# aux_source_directory(src/MTR MTR_SRCS)
# aux_source_directory(src SRCS)
# set(SPECIFIC_OUTPUT_DIR "${CMAKE_BUILD_TYPE}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}")

file(GLOB SHADERS
    shaders/*.fs
    shaders/*.vs
    shaders/*.gs)

# message(${LIBRARY_OUTPUT_PATH})
file(COPY ${SHADERS} DESTINATION ${EXECUTABLE_OUTPUT_DIRECTORY}${CMAKE_BUILD_TYPE}/shaders)

# add_library(MetaRay MTR/math_defs.cpp)

add_subdirectory(external/glad)
add_subdirectory(external/partio)
add_subdirectory(external/spdlog)

add_subdirectory(src/MTR)
add_subdirectory(src/MPM)
add_subdirectory(src/examples)



# set(SRCS
#     src/MTR/math_defs.cpp
#     main.cpp)

# message(${SRCS})

# set(ExecTarget ${PROJECT_NAME})
# add_executable(${ExecTarget} ${SRCS})

# target_link_libraries(${ExecTarget} PRIVATE glad)

# find_package(jsoncpp CONFIG REQUIRED)
# target_link_libraries(${ExecTarget} PRIVATE jsoncpp_lib jsoncpp_object)

# find_package(glfw3 CONFIG REQUIRED)
# target_link_libraries(${ExecTarget} PRIVATE glfw)

# find_package(Eigen3 CONFIG REQUIRED)
# target_link_libraries(${ExecTarget} PRIVATE Eigen3::Eigen)

# find_package(imgui CONFIG REQUIRED)
# target_link_libraries(${ExecTarget} PRIVATE imgui::imgui)

# find_package(spdlog CONFIG REQUIRED)
# target_link_libraries(${ExecTarget} PRIVATE spdlog::spdlog spdlog::spdlog_header_only)

