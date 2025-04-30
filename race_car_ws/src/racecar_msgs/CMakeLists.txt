cmake_minimum_required(VERSION 3.8)
project(racecar_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DetectedCone.msg
  msg/DetectedConeArray.msg
  msg/SemanticCell.msg
  msg/SemanticGrid.msg
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

ament_package()