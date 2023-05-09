cmake_minimum_required(VERSION 3.15)
project(example)

find_package(serial REQUIRED)

add_executable(main main.cpp)
# target_compile_definitions(main PRIVATE CXX17)
target_compile_options(main PRIVATE -g -std=c++17)
target_compile_definitions(main PRIVATE CXX17)
target_include_directories(main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include ${serial_INCLUDE_DIRS})
target_link_libraries(main
    PRIVATE
        ${serial_LIBRARIES}
)