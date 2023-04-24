add_library(${PROJECT_NAME} SHARED
        ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES})

add_subdirectory(backward-cpp)

add_executable(template_node examples/cpp/template_node.cpp ${BACKWARD_ENABLE})
add_backward(template_node)
target_link_libraries(template_node ${PROJECT_NAME})