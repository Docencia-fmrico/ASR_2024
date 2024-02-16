cmake_minimum_required(VERSION 3.8)
project(node_programming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
)

add_executable(simple src/simple.cpp)
ament_target_dependencies(simple ${dependencies})

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber ${dependencies})

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher ${dependencies})


# Using classes
add_executable(logger_class src/logger_class.cpp)
ament_target_dependencies(logger_class ${dependencies})

# Using a library
include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/node_programming/PublisherNode.cpp
  src/node_programming/SubscriberNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(publisher_class src/publisher_class.cpp)
ament_target_dependencies(publisher_class ${dependencies})
target_link_libraries(publisher_class ${PROJECT_NAME})

add_executable(subscriber_class src/subscriber_class.cpp)
ament_target_dependencies(subscriber_class ${dependencies})
target_link_libraries(subscriber_class ${PROJECT_NAME})

add_executable(pub_sub_st src/pub_sub_st.cpp)
ament_target_dependencies(pub_sub_st ${dependencies})
target_link_libraries(pub_sub_st ${PROJECT_NAME})

add_executable(pub_sub_mt src/pub_sub_mt.cpp)
ament_target_dependencies(pub_sub_mt ${dependencies})
target_link_libraries(pub_sub_mt ${PROJECT_NAME})

add_executable(pub_sub_sst src/pub_sub_sst.cpp)
ament_target_dependencies(pub_sub_sst ${dependencies})
target_link_libraries(pub_sub_sst ${PROJECT_NAME})

install(TARGETS
  simple
  logger
  publisher
  subscriber
  logger_class
  publisher_class
  subscriber_class
  pub_sub_st
  pub_sub_mt
  pub_sub_sst
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
