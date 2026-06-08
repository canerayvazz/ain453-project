cmake_minimum_required(VERSION 3.0.2)
project(pf_localization_real)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  duckietown_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pf_node_ros1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
