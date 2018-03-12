cmake_minimum_required(VERSION 2.8.3)
project(robotender_server)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  item.msg
  order.msg
  order_temp.msg
  location.msg
)

generate_messages(
  DEPENDENCIES
  robotender_server
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotender_server
  CATKIN_DEPENDS message_runtime rospy std_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/robotender_server/robotender_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)