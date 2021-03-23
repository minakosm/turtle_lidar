#include <iostream>
#include <thread>
#include <mutex>
#include <json/json.h>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster/impl/parsing.h"

class testPTP : public rclcpp::Node {
    private:
        std::string configFilePath;
        std::string host_ip, lidar_ip;
        ouster::sensor::lidar_mode lidarMode;
        ouster::sensor::timestamp_mode timestampMode;

        uint8_t* lidar_buf;
        std::shared_ptr<ouster::sensor::client> cli;

	uint64_t timestampCaptured;
        std_msgs::msg::Header headerMsg;

        int runDriver() {
            boost::property_tree::ptree pt;
	    boost::property_tree::ini_parser::read_ini(configFilePath, pt);
            
            host_ip = pt.get<std::string>("Lidar.hostIP");
            lidar_ip = pt.get<std::string>("Lidar.lidarIP");
            lidarMode = (ouster::sensor::lidar_mode)pt.get<int>("Lidar.lidarMode");
            timestampMode = (ouster::sensor::timestamp_mode)pt.get<int>("Lidar.timestampMode");

            cli = ouster::sensor::init_client(lidar_ip, host_ip, lidarMode, timestampMode);
            if (!cli) {
                RCLCPP_ERROR(this->get_logger(), "Failed to connect to client at: %s", lidar_ip);
                return 1;
            }

            lidar_buf = new uint8_t[ouster::sensor::lidar_packet_bytes_OS1_32 + 1];

            ouster::sensor::client_state st;
            while (rclcpp::ok()) {
                st = ouster::sensor::poll_client(*cli, 1);
                if (st & ouster::sensor::CLIENT_ERROR) {
                    RCLCPP_ERROR(this->get_logger(), "Lidar returned error status");
                    return 3;
                }
                else if (st & ouster::sensor::LIDAR_DATA) {
                    // auto k1 = std::chrono::high_resolution_clock::now();
                    headerMsg.stamp = rclcpp::Node::now();
		    // timestampCaptured = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
                    if (ouster::sensor::read_lidar_packet(*cli, lidar_buf, ouster::sensor::lidar_packet_bytes_OS1_32)) {
                        handleLidar();
                    }
                    else
                        RCLCPP_ERROR(this->get_logger(), "read_lidar_packet failed");
                    // auto k2 = std::chrono::high_resolution_clock::now();
                    // RCLCPP_INFO(this->get_logger(), "Lidar total time = %lluμs", std::chrono::duration_cast<std::chrono::microseconds>(k2 - k1).count());
                }
            }
            return -1;
        }

        void handleLidar() {
            const uint8_t* col_buf_check = ouster::sensor::impl::nth_col<32>(0, lidar_buf);
            uint64_t lidarTimestamp = ouster::sensor::impl::col_timestamp(col_buf_check);
	    timestampCaptured = (uint64_t)(headerMsg.stamp.sec)*1e+9 + (uint64_t)(headerMsg.stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "\nLiDAR Sensor Timestamp = %llu\nROS2 NodeNow Timestamp = %llu\nDifference = %llu", lidarTimestamp, timestampCaptured, (lidarTimestamp > timestampCaptured) ? (lidarTimestamp-timestampCaptured) : (timestampCaptured - lidarTimestamp));
        }
        
    public:
        testPTP(std::string pathToConfigFile = "/lidarConfig.ini");
};

testPTP::testPTP(std::string pathToConfigFile) : Node("testPTP_node") {
    configFilePath = pathToConfigFile;
    runDriver();
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);    
    std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_lidar");
    std::string pathToConfigFile = package_share_path + "/lidarConfig.ini";
    rclcpp::spin(std::make_shared<testPTP>(pathToConfigFile));
    rclcpp::shutdown();
    return 0;
}
