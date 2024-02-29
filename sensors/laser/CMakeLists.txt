cmake_minimum_required(VERSION 3.8)
project(laser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/laser/ObstacleDetectorNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(obstacle_detector src/obstacle_detector_main.cpp)
ament_target_dependencies(obstacle_detector ${dependencies})
target_link_libraries(obstacle_detector ${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  obstacle_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
