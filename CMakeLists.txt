#cmake_minimum_required(VERSION 2.8)
cmake_minimum_required(VERSION 3.5)
project(cmake_template)
message("start make cmake_template")

# 一定要加-march=native， 不然使用pcl的一些函数会报错
#add_definitions(-DCPP11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -march=native")

set(ALL_TARGET_LIBRARIES "")
include(cmake/global_defination.cmake)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)


file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "Thirdparty/*.cpp")
#list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


# lidar slam and localize
#add_executable(localize_map nodecpp/localize_map.cpp ${ALL_SRCS})
#target_link_libraries(localize_map ${ALL_TARGET_LIBRARIES})

