cmake_minimum_required(VERSION 3.14)
project(slam)

set(CMAKE_CXX_STANDARD 17)

# set(PYTHON3_NUMPY_INCLUDE_DIRS, /usr/lib/python3.7/site-packages/numpy/core/include)

find_package(PythonLibs REQUIRED)
find_package(Numpy REQUIRED)

include_directories(${PYTHON_INCLUDE_DIRS} ${PYTHON3_NUMPY_INCLUDE_DIRS})

add_executable(slam main.cpp Vec2d.cpp Vec2d.h feature_extraction.cpp feature_extraction.h)

target_link_libraries(slam ${PYTHON_LIBRARIES})