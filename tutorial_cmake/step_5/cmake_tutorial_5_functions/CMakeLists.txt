cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_5_functions)

find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/cmake_tutorial_5_functions/functions.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${OpenCV_LIBS}
  ${std_msgs_LIBS}
)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(${PROJECT_NAME}_LIBS
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${std_msgs_LIBS}
    CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
