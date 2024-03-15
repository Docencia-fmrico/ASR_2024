cmake_minimum_required(VERSION 3.5)
project(sump_bumpgo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)

set(dependencies
  rclcpp
  kobuki_ros_interfaces
  geometry_msgs
  rclcpp_cascade_lifecycle
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/sump_bumpgo/BumpGoBehavior.cpp
  src/sump_bumpgo/ForwardBehavior.cpp
  src/sump_bumpgo/AvoidBehavior.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(bumpgo src/bumpgo_main.cpp)
ament_target_dependencies(bumpgo ${dependencies})
target_link_libraries(bumpgo ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
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
