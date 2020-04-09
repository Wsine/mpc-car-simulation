cmake_minimum_required (VERSION 3.5)
project(mpc-car-simulation)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/MPC.cpp src/main.cpp)

include_directories(include)
include_directories(include/Eigen-3.3)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty/include)
link_directories(${CMAKE_SOURCE_DIR}/3rdparty/lib)
link_directories(${CMAKE_SOURCE_DIR}/3rdparty/lib64)

add_executable(mpc ${sources})
target_link_libraries(mpc ipopt z ssl uv uWS)

