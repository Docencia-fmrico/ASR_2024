cmake_minimum_required(VERSION 3.5)
project(cmake_tutorial_7_program)

find_package(cmake_tutorial_7_functions REQUIRED)

set(dependencies
    cmake_tutorial_7_functions
)

add_executable(program src/program.cpp)
ament_target_dependencies(program ${dependencies})

install(TARGETS
    program
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
