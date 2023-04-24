#include <iostream>
#include <string>
#include "glog/logging.h"
#include "file_manager.hpp"
#include <Eigen/Dense>
#include "global_definition/global_definition.h"
#include "System.h"
#include "lie_utils.h"

using namespace std;

///////主函数
int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = cmake_template::WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;
    FileManager::CreateDirectory(FLAGS_log_dir);

    System system;

    LOG(INFO)<<"Return.";

    return 0;
}
