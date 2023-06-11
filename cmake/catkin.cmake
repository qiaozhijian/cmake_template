find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        roslib
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        pcl_ros
        eigen_conversions
        )
catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)
list(APPEND ALL_TARGET_LIBRARIES ${catkin_LIBRARIES})
message(STATUS "ALL_TARGET_LIBRARIES: ${ALL_TARGET_LIBRARIES}")
