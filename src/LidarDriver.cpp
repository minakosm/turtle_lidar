#include "LidarDriver.h"
#include "PointcloudProcessing.h"
#include "os1.h"
#include "os1_packet.h"

#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <json/json.h>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <chrono>
#include <cstdint>

#ifndef PI
#define PI 3.141593
#endif

LidarDriver::LidarDriver() : pointcloudProcessor() {
    readSettingsFromINI();
    endFlag = false;
    height = 32;
    counter = 0;
    scan_counter = 0;
    switch(lidarMode) {
        case ouster::OS1::MODE_512x10:
            width = 512;
            rotationRate = 10;
            packetsPerScan = 32;
            break;
        case ouster::OS1::MODE_512x20:
            width = 512;
            rotationRate = 20;
            packetsPerScan = 32;
            break;
        case ouster::OS1::MODE_1024x10:
            width = 1024;
            rotationRate = 10;
            packetsPerScan = 64;
            break;
        case ouster::OS1::MODE_1024x20:
            width = 1024;
            rotationRate = 20;
            packetsPerScan = 64;
            break;
        case ouster::OS1::MODE_2048x10:
            width = 2048;
            rotationRate = 10;
            packetsPerScan = 128;
            break;
        default:
            std::cout << "Invalid lidar_mode\n";
            return;
    }
    times_buffer.resize(width);
    ranges_buffer.resize(width * height);
    intensities_buffer.resize(width * height);

    times_process_buffer.resize(width);
    ranges_process_buffer.resize(width * height);
    intensities_process_buffer.resize(width * height);
    // std::cout << "Width = " << width 
    //           << "\nHeight = " << height 
    //           << "\nRotation Rate = " << rotationRate 
    //           << "\nPackets Per Scan = " << packetsPerScan 
    //           << "\nCounter = " << unsigned(counter) 
    //           << "\nScan Counter = " << scan_counter 
    //           << std::endl;
}

LidarDriver::~LidarDriver() {
}

void LidarDriver::readSettingsFromINI(std::string pathToIniFile) {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(pathToIniFile, pt);

    host_ip = pt.get<std::string>("Lidar.hostIP");
    lidar_ip = pt.get<std::string>("Lidar.lidarIP");
    lidarMode = (ouster::OS1::lidar_mode)pt.get<int>("Lidar.lidarMode");
    timestampMode = (ouster::OS1::timestamp_mode)pt.get<int>("Lidar.timestampMode");
    // std::cout << "lidarMode = " << lidarMode << "\ntimestampMode = " << timestampMode << std::endl;
}

int LidarDriver::run_driver() {
    cli = ouster::OS1::init_client(lidar_ip, host_ip, lidarMode, timestampMode);
    if (!cli) {
        std::cout << "Failed to connect to client at: " << lidar_ip << std::endl;
        return 1;
    }
    std::cout << "Lidar Driver Initializing\n";
    initialize();
    std::cout << "Lidar Driver Initialized\n";

    uint8_t lidar_buf[ouster::OS1::lidar_packet_bytes + 1];

    ouster::OS1::client_state st;
    while (!endFlag) {
        st = ouster::OS1::poll_client(*cli, 1);
        if (st & ouster::OS1::ERROR) {
            std::cout << "Lidar returned error status\n";
            return 3;
        }
        else if (st & ouster::OS1::LIDAR_DATA) {
            if (ouster::OS1::read_lidar_packet(*cli, lidar_buf)) {
                handle_lidar(lidar_buf);
            }
            else
                std::cout << "read_lidar_packet failed\n";
        }
    }
}

void LidarDriver::initialize() {
    beam_azim_angles = Eigen::VectorXf::Zero(height);
    beam_alt_angles = Eigen::VectorXf::Zero(height);
    for(int i = 32; i < 64; i++) {
        beam_azim_angles(i-32) = cli->meta["beam_azimuth_angles"][i].asFloat();
        beam_alt_angles(i-32) = cli->meta["beam_altitude_angles"][i].asFloat();
    }

    x_lut = Eigen::Matrix <float, Eigen::Dynamic, 1>::Zero(width * height);
    y_lut = Eigen::Matrix <float, Eigen::Dynamic, 1>::Zero(width * height);
    z_lut = Eigen::Matrix <float, Eigen::Dynamic, 1>::Zero(width * height);

    for (int i = 0; i < width; i++) {
        float azimAngle_0 = 2.0 * PI * i / width;
        for (int j = 0; j < height; j++) {
            float azimAngle = (beam_azim_angles(j) * PI / 180.0f) + azimAngle_0;
            x_lut(i*height + j) = std::cos(beam_alt_angles(j) * PI / 180.0f) * std::cos(azimAngle);
            y_lut(i*height + j) = -std::cos(beam_alt_angles(j) * PI / 180.0f) * std::sin(azimAngle);
            z_lut(i*height + j) = std::sin(beam_alt_angles(j) * PI / 180.0f);

            // x_lut(i*height + j) = std::sin(beam_alt_angles(j) * PI / 180.0f) * std::cos(azimAngle);
            // y_lut(i*height + j) = std::sin(beam_alt_angles(j) * PI / 180.0f) * std::sin(azimAngle);
            // z_lut(i*height + j) = std::cos(beam_alt_angles(j) * PI / 180.0f);
        }
    }
}

void LidarDriver::handle_lidar(uint8_t* lidar_buf) {
    // std::cout << "Handle Lidar Start\n";
    // mutex lock
	//#pragma omp parallel for ordered num_threads(2)
	for(int i = counter; i < (counter + ouster::OS1::columns_per_buffer); i++) {
	    const uint8_t* col_buf = ouster::OS1::nth_col(i-counter, lidar_buf);
	    if(ouster::OS1::col_valid(col_buf)) {
	        times_buffer[i] = ouster::OS1::col_timestamp(col_buf);
	        const uint8_t* px;
	        for(int j = 0; j < height; j++) {
	            px = ouster::OS1::nth_px(j+height, col_buf);
	            ranges_buffer[i*height+j] = ouster::OS1::px_range(px);
	            intensities_buffer[i*height+j] = ouster::OS1::px_reflectivity(px);
	        }
	    }
	}
    // // mutex unlock
	counter++;
    // std::cout << "Counter = " << unsigned(counter) << std::endl;
	if(counter == packetsPerScan) {
	    counter = 0;
	    // mutex process buffer lock
	    times_buffer.swap(times_process_buffer);
	    ranges_buffer.swap(ranges_process_buffer);
	    intensities_buffer.swap(intensities_process_buffer);
        // mutex process buffer unlock
	    std::thread t(&LidarDriver::handle_lidar_scan, this);
	    t.detach();
	}
    // std::cout << "Handle Lidar End\n";
}

void LidarDriver::handle_lidar_scan() {
    std::cout << "Handle Lidar Scan Start\n";
    
    Eigen::Matrix <uint32_t, Eigen::Dynamic, 1> ranges = Eigen::Map<Eigen::Matrix <uint32_t, Eigen::Dynamic, 1>>(ranges_process_buffer.data(), ranges_process_buffer.size());
    std::ofstream file("ranges.txt");
    if (file.is_open())
        file << ranges;
    file.close();
    
    std::cout << "Handle Lidar Scan End\n";
    this->endFlag = true;
}

void LidarDriver::handle_imu(uint8_t* imu_buf) {

}
