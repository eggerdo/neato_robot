cmake_minimum_required(VERSION 2.8.3)
project(neato_node)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetVacuum.srv
  SetBrush.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(DEPENDS)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
