cmake_minimum_required(VERSION 3.4)
project(slam)

set(CMAKE_CXX_STANDARD 11)

find_package(PythonLibs 2.7)

add_executable(slam main.cpp Vec2d.cpp Vec2d.h feature_extraction.cpp feature_extraction.h)

target_include_directories(slam PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(slam ${PYTHON_LIBRARIES})