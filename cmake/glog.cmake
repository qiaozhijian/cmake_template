find_package (glog 0.6.0 REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES glog::glog)
