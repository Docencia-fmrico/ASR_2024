cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_1)

include_directories(include)

add_executable(program 
  src/program.cpp
  src/functions.cpp
)
