cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_6_program)

find_package(cmake_tutorial_6_functions REQUIRED)

add_executable(program src/program.cpp)
ament_target_dependencies(program cmake_tutorial_6_functions)

install(TARGETS
    program
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
