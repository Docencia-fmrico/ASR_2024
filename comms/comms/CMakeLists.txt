cmake_minimum_required(VERSION 3.8)
project(comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  nav2_msgs
  tf2_geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/comms/ServiceClient.cpp
  src/comms/ServiceClient2.cpp
  src/comms/ServiceServer.cpp
  src/comms/ActionClient.cpp
  src/comms/ActionServer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(service_client_main src/service_client_main.cpp)
ament_target_dependencies(service_client_main ${dependencies})
target_link_libraries(service_client_main ${PROJECT_NAME})

add_executable(service_server_main src/service_server_main.cpp)
ament_target_dependencies(service_server_main ${dependencies})
target_link_libraries(service_server_main ${PROJECT_NAME})

add_executable(action_client_main src/action_client_main.cpp)
ament_target_dependencies(action_client_main ${dependencies})
target_link_libraries(action_client_main ${PROJECT_NAME})

add_executable(action_server_main src/action_server_main.cpp)
ament_target_dependencies(action_server_main ${dependencies})
target_link_libraries(action_server_main ${PROJECT_NAME})


install(TARGETS
  ${PROJECT_NAME}
  service_client_main
  service_server_main
  action_client_main
  action_server_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
