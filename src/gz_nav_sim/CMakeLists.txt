cmake_minimum_required(VERSION 3.8)
project(gz_nav_sim)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/da3_depth_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds maps models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
