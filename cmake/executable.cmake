add_library(${PROJECT_NAME} SHARED
        ${ALL_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${ALL_TARGET_LIBRARIES})

add_executable(template_node node/template_node.cpp)
target_link_libraries(template_node ${PROJECT_NAME})