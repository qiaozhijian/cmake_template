#include <iostream>
#include <string>
#include <glog/logging.h>
#include "config.h"
#include "System.h"

using namespace std;

///////主函数
int main(int argc, char **argv) {

    if (argc != 2) {
        std::cout << "Usage: kitti_bm config_file" << std::endl;
        return -1;
    }
    std::string config_path = config::project_path + "/configs/" + argv[1];
    InitGLOG(string(argv[1]).substr(0, string(argv[1]).find_last_of(".")));
    config::readParameters(config_path);

    System system;

    LOG(INFO)<<"Return.";

    return 0;
}
