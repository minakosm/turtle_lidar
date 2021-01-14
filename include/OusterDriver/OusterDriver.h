#ifndef OUSTER_DRIVER_H
#define OUSTER_DRIVER_H

#include <eigen3/Eigen/Dense>
#include <json/json.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>

#include "PointcloudProcessing.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "turtle_interfaces/msg/ouster_imu.hpp"

#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster/impl/parsing.h"

#ifndef PI
#define PI 3.141593
#endif

class OusterDriver : public rclcpp::Node {
    private:
        ouster::sensor::lidar_mode lidarMode;
        ouster::sensor::timestamp_mode timestampMode;
        std::string lidar_ip, host_ip;

        int height;
        int width;
        int rotationRate;
        int packetsPerScan;

        uint8_t counter;
        int scan_counter;

        std::shared_ptr<ouster::sensor::client> cli;
        uint8_t* lidar_buf;
        uint8_t* imu_buf;
        
        std::vector<float> beam_azim_angles;
        std::vector<float> beam_alt_angles;
        
        std::vector<uint64_t> times_buffer;
        std::vector<uint32_t> ranges_buffer;
        std::vector<uint16_t> intensities_buffer;
        
        std::vector<uint64_t> times_process_buffer;
        std::vector<uint32_t> ranges_process_buffer;
        std::vector<uint16_t> intensities_process_buffer;
        
        std::vector<float> x_lut;
        std::vector<float> y_lut;
        std::vector<float> z_lut;

        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 1>> X;
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 1>> Y;
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 1>> Z;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawPointcloudPublisher;
        sensor_msgs::msg::PointCloud2 rawPointcloudMsg;
        bool publishRaw;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr conesDetectedPublisher;
        sensor_msgs::msg::PointCloud2 conesDetectedMsg;
        bool publishCones;

        rclcpp::Publisher<turtle_interfaces::msg::OusterImu>::SharedPtr imuPublisher;
        turtle_interfaces::msg::OusterImu imuMsg;
        bool imuMode;

        std_msgs::msg::Header a;

        PointcloudProcessing pointcloudProcessor;

    public:
        OusterDriver(std::string configFilePath = "./config.ini",
                     std::string coneTrainXFilePath = "./example/simConeTrainDataX.txt",
                     std::string coneTrainYFilePath = "./example/simConeTrainDataY.txt");
        ~OusterDriver();

        void readSettingsFromINI(std::string pathToIniFile);
        int runDriver();
        void initialize();
        void initializePublishers();
        void handleLidar();
        void handleImu();
        void handleLidarScan();
        void publishRawPointcloud();
        void publishImu();
};

#endif // OUSTER_DRIVER_H_INCLUDED
