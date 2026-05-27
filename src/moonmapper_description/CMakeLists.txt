cmake_minimum_required(VERSION 3.8)
project(moonmapper_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# -----------------------------------------------------------------
# Gazebo Sim 8 system plugin (optional)
# -----------------------------------------------------------------
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET)

if(gz-sim8_FOUND AND gz-plugin2_FOUND)
  message(STATUS "Building moonmapper_rocker_bogie_differential plugin for gz-sim8")
  add_library(moonmapper_rocker_bogie_differential SHARED
    gz_plugins/src/RockerBogieDifferential.cc
  )
  target_link_libraries(moonmapper_rocker_bogie_differential
    PRIVATE gz-sim8::gz-sim8 gz-plugin2::gz-plugin2)
  install(TARGETS moonmapper_rocker_bogie_differential
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
else()
  message(WARNING
    "gz-sim8 / gz-plugin2 not found; skipping Gazebo plugin build.")
endif()

install(DIRECTORY
  launch
  urdf
  rviz
  meshes
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
