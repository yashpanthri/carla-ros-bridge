cmake_minimum_required(VERSION 3.8)
project(monocular_depth_pkg)

# bring in the default ROS 2 message generators
find_package(rosidl_default_generators REQUIRED)

# tell rosidl to generate code for our message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DepthStamped.msg"
  DEPENDENCIES std_msgs       # DepthStamped.msg uses std_msgs/Header
)

# so that downstream packages find our python message module
ament_export_dependencies(rosidl_default_runtime)
ament_package()
