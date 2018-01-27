cmake_minimum_required(VERSION 2.8.3)
project(strokeRehabSystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

# Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES UDPMessage.srv)


generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strokeRehab
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
