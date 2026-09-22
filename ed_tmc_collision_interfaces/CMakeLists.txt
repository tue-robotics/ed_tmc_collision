cmake_minimum_required(VERSION 3.8)
project(ed_tmc_collision_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tmc_manipulation_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCollisionEnvironment.srv"
  DEPENDENCIES
  tmc_manipulation_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
