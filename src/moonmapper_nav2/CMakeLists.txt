cmake_minimum_required(VERSION 3.16)
project(moonmapper_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
  scripts/frontier_explorer
  scripts/nav2_map_ready_wait_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
