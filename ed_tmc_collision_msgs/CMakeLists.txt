cmake_minimum_required(VERSION 3.5)
project(ed_tmc_collision_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  tmc_manipulation_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  GetCollisionEnvironment.srv
)

generate_messages(
  DEPENDENCIES
  tmc_manipulation_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ed_tmc_collision_msgs
  CATKIN_DEPENDS message_runtime tmc_manipulation_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2")
endif()
