cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_4)

find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(functions_cmake_tutorial_4 SHARED src/cmake_tutorial_4/functions.cpp)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  functions_cmake_tutorial_4
  ${OpenCV_LIBS}
  ${std_msgs_LIBS}
)
