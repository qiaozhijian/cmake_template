cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_definition.cmake)
include(cmake/glog.cmake)
include(cmake/eigen.cmake)
include(cmake/pcl.cmake)
include(cmake/yaml.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")
add_subdirectory(Thirdparty/robot_utils)

add_library(${PROJECT_NAME} SHARED
        ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES}
        ROBOT_UTILS::robot_utils
        )

add_executable(main examples/cpp/template.cpp)
target_link_libraries(main ${PROJECT_NAME})

