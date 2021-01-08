#include "OusterDriver.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);    
    rclcpp::spin(std::make_shared<OusterDriver>());
    rclcpp::shutdown();
    return 0;
}