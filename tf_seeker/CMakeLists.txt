cmake_minimum_required(VERSION 3.8)
project(tf_seeker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
    rclcpp
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/tf_seeker/TFPublisherNode.cpp
  src/tf_seeker/TFSeekerNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher ${dependencies})
target_link_libraries(tf_publisher ${PROJECT_NAME})

add_executable(seeker src/seeker.cpp)
ament_target_dependencies(seeker ${dependencies})
target_link_libraries(seeker ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  tf_publisher
  seeker
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

ament_export_dependencies(${dependencies})
ament_package()
