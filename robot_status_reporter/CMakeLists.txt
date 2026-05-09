cmake_minimum_required(VERSION 3.8)
project(robot_status_reporter)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DeliveryStatus.msg"
  DEPENDENCIES geometry_msgs
)

install(PROGRAMS
  robot_status_reporter/status_aggregator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME status_aggregator
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
