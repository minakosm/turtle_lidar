#include "ousterDriver.h"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" 

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    if(argc == 1) {
        std::string pathToConfigFile = argv[1];
        rclcpp::spin(std::make_shared<OusterDriver>(pathToConfigFile));
    }else{
        std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_lidar");
        std::string pathToConfigFile = package_share_path + "/lidarConfig.ini";
        rclcpp::spin(std::make_shared<OusterDriver>(pathToConfigFile));
    }
    rclcpp::shutdown();
    return 0;
    }
