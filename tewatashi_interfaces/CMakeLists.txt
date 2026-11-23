cmake_minimum_required(VERSION 3.22)
project(tewatashi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Segment.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
