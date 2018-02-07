cmake_minimum_required(VERSION 2.8.3)
project(strokeRehabSystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)
catkin_python_setup()

# Generate services in the 'srv' folder

add_service_files(DIRECTORY srv FILES UDPMessage.srv)
add_service_files(DIRECTORY srv FILES ReturnJointStates.srv)

add_message_files(DIRECTORY msg FILES udpMessage.msg)


generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  INCLUDE_DIRS src/tools


)
install(PROGRAMS scripts/hid2udp.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
