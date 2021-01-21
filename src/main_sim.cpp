#include "SimDriver.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    if(argc == 4) {
        std::string pathToConfigFile = argv[1];
        std::string pathToTrainXFile = argv[2];
        std::string pathToTrainYFile = argv[3];
        rclcpp::spin(std::make_shared<SimDriver>(pathToConfigFile, pathToTrainXFile, pathToTrainYFile));
    } else {
        std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_lidar");
        std::string pathToConfigFile = package_share_path + "/lidarConfig.ini";
        std::string pathToTrainXFile = package_share_path + "/simConeTrainDataX.txt";
        std::string pathToTrainYFile = package_share_path + "/simConeTrainDataY.txt";
        rclcpp::spin(std::make_shared<SimDriver>(pathToConfigFile, pathToTrainXFile, pathToTrainYFile));
    }
    rclcpp::shutdown();
    return 0;
}