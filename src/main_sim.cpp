#include "SimDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    if(argc == 2) {
        std::string pathToConfigFile = argv[1];
        rclcpp::spin(std::make_shared<SimDriver>(pathToConfigFile));
    } else if(argc == 3) {
        std::cout << "You specified path for only one cone train data file\n";
    } else if(argc > 3) {
        std::string pathToConfigFile = argv[1];
        std::string pathTotrainXFile = argv[2];
        std::string pathTotrainYFile = argv[3];
        rclcpp::spin(std::make_shared<SimDriver>(pathToConfigFile, pathTotrainXFile, pathTotrainYFile));
    } else {
        rclcpp::spin(std::make_shared<SimDriver>());
    }
    std::cout << "Shuttind down\n";
    rclcpp::shutdown();
    return 0;
}