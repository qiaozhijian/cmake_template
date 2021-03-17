find_package(OpenCV REQUIRED)
add_definitions(-DUSE_OPENCV)
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})