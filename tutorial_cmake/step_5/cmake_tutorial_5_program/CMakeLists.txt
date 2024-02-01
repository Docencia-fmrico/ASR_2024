cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_5_program)

include_directories(
  ${cmake_tutorial_5_functions_INCLUDE_DIRS}
)

add_executable(program  src/program.cpp)
target_link_libraries(program 
  ${cmake_tutorial_5_functions_LIBS}
)
