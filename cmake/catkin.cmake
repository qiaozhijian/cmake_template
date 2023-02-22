
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        rospy
        tf
        roslib
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs
)
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

list(APPEND ALL_TARGET_LIBRARIES ${catkin_LIBRARIES})
