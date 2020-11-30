#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <eigen3/Eigen/Dense>
#include <json/json.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <cstdint>
#include <memory>

#include "PointcloudProcessing.h"
#include "os1.h"
#include "os1_packet.h"

class LidarDriver {
    private:
        ouster::OS1::lidar_mode lidarMode;
        ouster::OS1::timestamp_mode timestampMode;
        std::string lidar_ip, host_ip;

        int height;
        int width;
        int rotationRate;
        int packetsPerScan;

        uint8_t counter;
        int scan_counter;
        bool endFlag;

        std::shared_ptr<ouster::OS1::client> cli;
        
        Eigen::VectorXf beam_azim_angles;
        Eigen::VectorXf beam_alt_angles;
        
        std::vector<uint64_t> times_buffer;
        std::vector<uint32_t> ranges_buffer;
        std::vector<uint16_t> intensities_buffer;
        
        std::vector<uint64_t> times_process_buffer;
        std::vector<uint32_t> ranges_process_buffer;
        std::vector<uint16_t> intensities_process_buffer;
        
        Eigen::Matrix <float, Eigen::Dynamic, 1> x_lut;
        Eigen::Matrix <float, Eigen::Dynamic, 1> y_lut;
        Eigen::Matrix <float, Eigen::Dynamic, 1> z_lut;

        PointcloudProcessing pointcloudProcessor;
            
    public:
        LidarDriver();
        ~LidarDriver();

        void readSettingsFromINI(std::string pathToIniFile = "../config.ini");
        int run_driver();
        void initialize();
        void handle_lidar(uint8_t* lidar_buf);
        void handle_imu(uint8_t* imu_buf);
        void handle_lidar_scan();
};

#endif // LIDAR_DRIVER_H_INCLUDED
