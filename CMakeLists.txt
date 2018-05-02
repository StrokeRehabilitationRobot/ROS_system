cmake_minimum_required(VERSION 2.8.3)
project(strokeRehabSystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  genmsg
)
catkin_python_setup()

# Generate services in the 'srv' folder

add_service_files(DIRECTORY srv FILES UDPMessage.srv ReturnJointStates.srv)
# add_service_files(DIRECTORY srv FILES ReturnJointStates.srv)

add_message_files(DIRECTORY msg FILES

udpMessage.msg
hapticForce.msg
PlayerState.msg
udpTorque.msg
)


generate_messages(DEPENDENCIES
std_msgs
geometry_msgs
)


catkin_package(
  INCLUDE_DIRS src/tools
  INCLUDE_DIRS src/games
  INCLUDE_DIRS src/controllers
  INCLUDE_DIRS src/tests
  INCLUDE_DIRS src/models
  INCLUDE_DIRS src/controller_nodes
  INCLUDE_DIRS src/lib/dmp/Python
)
install(PROGRAMS scripts/hid2udp.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)