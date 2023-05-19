cmake_minimum_required(VERSION 3.5)
project(cmake_template)

message("start make cmake_template")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_CXX_STANDARD 17)

set(ALL_TARGET_LIBRARIES "")
include(cmake/boost.cmake)
include(cmake/global_definition.cmake)
include(cmake/glog.cmake)
include(cmake/eigen.cmake)
include(cmake/OpenCV.cmake)
include(cmake/catkin.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

file(GLOB_RECURSE ALL_SRCS "src/*.cpp")

add_subdirectory(Thirdparty/robot_utils)

add_library(${PROJECT_NAME} SHARED
        ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES}
        ROBOT_UTILS::robot_utils
        )

#add_subdirectory(backward-cpp)
#add_executable(template_node examples/cpp/template_node.cpp ${BACKWARD_ENABLE})
add_executable(template_node examples/cpp/template_node.cpp)
#add_backward(template_node)
target_link_libraries(template_node ${PROJECT_NAME})

