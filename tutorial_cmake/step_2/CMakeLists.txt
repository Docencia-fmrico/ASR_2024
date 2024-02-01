cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_2)

include_directories(include)

add_library(functions_cmake_tutorial_2 SHARED src/cmake_tutorial_2/functions.cpp)

add_executable(program  src/program.cpp)
target_link_libraries(program functions_cmake_tutorial_2)
