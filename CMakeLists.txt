cmake_minimum_required(VERSION 3.28)
project(untitled)

set(CMAKE_CXX_STANDARD 11)
add_subdirectory(extern/pybind11)

add_executable(untitled main.cpp)
target_link_libraries(untitled PRIVATE pybind11::embed)
