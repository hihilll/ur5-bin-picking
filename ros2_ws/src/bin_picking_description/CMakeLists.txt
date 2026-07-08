cmake_minimum_required(VERSION 3.8)
project(bin_picking_description)

find_package(ament_cmake REQUIRED)

# config/ 内含从 ur_description 拷入的 UR 本体参数(joint_limits/kinematics/
# physical/visual)。UR 驱动把这些路径写死为 <description_package>/config/<ur_type>/，
# 本包既然接管了 description_package，就必须自带这些文件。
install(DIRECTORY meshes scene urdf srdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
