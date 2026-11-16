# tuba-sim bridge: Gazebo <-> Tuba firmware (software/hardware-in-the-loop)
#
#   cmake -S bridge -B bridge/build && cmake --build bridge/build
#   (or scripts/build_bridge.sh)
cmake_minimum_required(VERSION 3.16)
project(tuba_sil_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Gazebo Jetty (gz-sim 10): unversioned cmake package names.
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_executable(sil_bridge sil_bridge.cpp)
target_link_libraries(sil_bridge PRIVATE gz-transport::gz-transport gz-msgs::gz-msgs)
target_compile_options(sil_bridge PRIVATE -Wall -Wextra)
