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

        std::shared_ptr<ouster::OS1::client> cli;
        uint8_t* lidar_buf;
        
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

        PointcloudProcessing pointcloudProcessor;
            
    public:
        LidarDriver();
        ~LidarDriver();

        void readSettingsFromINI(std::string pathToIniFile = "../config.ini");
        int run_driver();
        void initialize();
        void handle_lidar();
        void handle_lidar_scan();
};

#endif // LIDAR_DRIVER_H_INCLUDED
