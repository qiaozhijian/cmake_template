cmake_minimum_required(VERSION 3.5)
project(ros_package_template)

message("start make ros_package_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_defination.cmake)
include(cmake/YAML.cmake)
include(cmake/geographic.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/catkin.cmake)
include(cmake/OpenCV.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_executable(point_cloud node/point_cloud.cpp ${ALL_SRCS})
target_link_libraries(point_cloud ${ALL_TARGET_LIBRARIES})

