cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_defination.cmake)
include(cmake/geographic.cmake)
include(cmake/glog.cmake)
include(cmake/eigen.cmake)
include(cmake/OpenCV.cmake)
include(cmake/catkin.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

include(cmake/executable.cmake)

