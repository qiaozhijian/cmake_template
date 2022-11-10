#include <iostream>
#include <string>
#include "glog/logging.h"
#include "file_manager.hpp"
#include <Eigen/Core>
#include "global_defination/global_defination.h"
#include "System.h"
#include "config.h"

using namespace std;

///////主函数
int main(int argc, char **argv) {

    ros::init(argc, argv, "receive_pcl");
    ros::NodeHandle nh("~");

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = ros_package_template::WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);

    Config::readConfig();

    ros_package_template::System system(nh);

    ros::spin();

    return 0;
}
