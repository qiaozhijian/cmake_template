#include "file_manager.h"
#include <boost/filesystem.hpp>

using namespace std;

namespace FileManager{
    bool CreateFile(std::ofstream &ofs, std::string file_path) {
        ofs.close();
        boost::filesystem::remove(file_path.c_str());

        ofs.open(file_path.c_str(), std::ios::out);
        if (!ofs) {
            std::cerr << "Cannot create file: " << file_path << std::endl;
            return false;
        }

        return true;
    }

    bool InitDirectory(std::string directory_path) {
        if (boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::remove_all(directory_path);
        }

        return CreateDirectory(directory_path);
    }

    bool CreateDirectory(std::string directory_path) {
        if (!boost::filesystem::is_directory(directory_path)) {
            boost::filesystem::create_directory(directory_path);
        }

        if (!boost::filesystem::is_directory(directory_path)) {
            std::cerr << "Cannot create directory: " << directory_path << std::endl;
            return false;
        }

        return true;
    }

    int CountFiles(std::string directory_path, std::string suffix){
        int count = 0;
        boost::filesystem::path directory(directory_path);
        boost::filesystem::directory_iterator end_iter;
        for (boost::filesystem::directory_iterator iter(directory); iter != end_iter; iter++) {
            if (boost::filesystem::is_regular_file(iter->status())) {
                if (iter->path().extension() == suffix) {
                    count++;
                }
            }
        }
        return count;
    }
}