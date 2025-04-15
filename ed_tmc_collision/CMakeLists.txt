cmake_minimum_required(VERSION 3.5)
project(ed_tmc_collision)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

find_package(catkin REQUIRED COMPONENTS
  ed
  ${PROJECT_NAME}_msgs
  geolib2
  geometry_msgs
  roscpp
  tmc_manipulation_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_plugin
  CATKIN_DEPENDS ed ${PROJECT_NAME}_msgs roscpp
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
# include
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(boost_http_server src/boost_http_server.cpp src/boost_http_server.h)
target_link_libraries(boost_http_server ${catkin_LIBRARIES})

add_library(http_server src/http_server.cpp src/http_server.h)
target_link_libraries(http_server boost_http_server ${catkin_LIBRARIES})


add_library(${PROJECT_NAME}_plugin
  src/tmc_collision_plugin.cpp
  src/tmc_collision_plugin.h
)
target_link_libraries(${PROJECT_NAME}_plugin http_server ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_plugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    boost_http_server
    ${PROJECT_NAME}_plugin
    http_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2")
endif()
