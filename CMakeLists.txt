cmake_minimum_required(VERSION 3.5)
project(rover_nav)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
)
ament_python_install_package(scripts)
install(PROGRAMS
  scripts/obstacle_detector_node.py
  scripts/pure_pursuit_avoidance_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
