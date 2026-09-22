cmake_minimum_required(VERSION 3.8)
project(ed_tmc_collision)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Werror=all -Wextra -Werror=extra)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(ed REQUIRED)
find_package(ed_tmc_collision_interfaces REQUIRED)
find_package(geolib2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_geometric_shapes_msgs REQUIRED)
find_package(tmc_manipulation_msgs REQUIRED)

add_library(boost_http_server SHARED src/boost_http_server.cpp src/boost_http_server.h)
target_link_libraries(boost_http_server PUBLIC Boost::filesystem rclcpp::rclcpp)

add_library(http_server SHARED src/http_server.cpp src/http_server.h)
target_link_libraries(http_server PUBLIC boost_http_server)

add_library(${PROJECT_NAME}_plugin SHARED
  src/tmc_collision_plugin.cpp
  src/tmc_collision_plugin.h
)
target_link_libraries(${PROJECT_NAME}_plugin PUBLIC
  ed::ed_core
  ${ed_tmc_collision_interfaces_TARGETS}
  geolib2::geolib
  ${geometry_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${tmc_geometric_shapes_msgs_TARGETS}
  ${tmc_manipulation_msgs_TARGETS}
  http_server
)

pluginlib_export_plugin_description_file(ed plugins.xml)

#############
## Install ##
#############

install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

install(
  TARGETS
    boost_http_server
    ${PROJECT_NAME}_plugin
    http_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Boost
  ed
  ed_tmc_collision_interfaces
  geolib2
  geometry_msgs
  pluginlib
  rclcpp
  tf2
  tf2_ros
  tmc_geometric_shapes_msgs
  tmc_manipulation_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
