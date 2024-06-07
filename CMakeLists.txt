cmake_minimum_required(VERSION 2.8.3)
project(ros_ati_mini45)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs  # Include any other message dependencies
  message_generation
)

catkin_python_setup()


add_message_files(DIRECTORY msg FILES ForceMeasurement.msg ZeroingDuration.msg)


generate_messages(DEPENDENCIES std_msgs)

catkin_install_python(PROGRAMS
  src/ros_ati_mini45_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros_ati_mini45
)

install(FILES
  launch/main.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros_ati_mini45
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


