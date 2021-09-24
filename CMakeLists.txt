cmake_minimum_required(VERSION 2.8.3)
project(status_lights_ben)
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation 
  std_msgs)
catkin_python_setup()

catkin_install_python(PROGRAMS
    src/status_lights_ben/__init__.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/status_lights_ben)

generate_messages(DEPENDENCIES std_msgs)

