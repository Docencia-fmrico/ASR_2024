cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_3)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(functions_cmake_tutorial_3 SHARED src/cmake_tutorial_3/functions.cpp)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  functions_cmake_tutorial_3
  ${OpenCV_LIBS}
)
