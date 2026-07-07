cmake_minimum_required(VERSION 3.8)
project(bin_picking_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes scene urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
