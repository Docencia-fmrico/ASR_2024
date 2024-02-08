cmake_minimum_required(VERSION 3.5)
project(fsm_bumpgo)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  kobuki_ros_interfaces
  geometry_msgs
)

include_directories(include)

add_executable(bumpgo
  src/fsm_bumpgo/BumpGoNode.cpp
  src/bumpgo_main.cpp
)
ament_target_dependencies(bumpgo ${dependencies})

install(TARGETS
  bumpgo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
