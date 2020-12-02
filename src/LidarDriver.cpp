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
}

LidarDriver::~LidarDriver() {
    delete[] lidar_buf;
}

void LidarDriver::readSettingsFromINI(std::string pathToIniFile) {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(pathToIniFile, pt);

    host_ip = pt.get<std::string>("Lidar.hostIP");
    lidar_ip = pt.get<std::string>("Lidar.lidarIP");
    lidarMode = (ouster::OS1::lidar_mode)pt.get<int>("Lidar.lidarMode");
    timestampMode = (ouster::OS1::timestamp_mode)pt.get<int>("Lidar.timestampMode");
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

    lidar_buf = new uint8_t[ouster::OS1::lidar_packet_bytes + 1];

    ouster::OS1::client_state st;
    while (true) {
        st = ouster::OS1::poll_client(*cli, 1);
        if (st & ouster::OS1::ERROR) {
            std::cout << "Lidar returned error status\n";
            return 3;
        }
        else if (st & ouster::OS1::LIDAR_DATA) {
            if (ouster::OS1::read_lidar_packet(*cli, lidar_buf)) {
                auto a1 = std::chrono::high_resolution_clock::now();
                handle_lidar();
                auto a2 = std::chrono::high_resolution_clock::now();
                if(counter % 10 == 0)
                    std::cout << "Handle time = " << std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count() << "μs" << std::endl;
            }
            else
                std::cout << "read_lidar_packet failed\n";
        }
    }
}

void LidarDriver::initialize() {
    beam_azim_angles.resize(height);
    beam_alt_angles.resize(height);
    for(int i = height; i < 64; i++) {
        beam_azim_angles[i-height] = cli->meta["beam_azimuth_angles"][i].asFloat();
        beam_alt_angles[i-height] = cli->meta["beam_altitude_angles"][i].asFloat();
    }

    x_lut.resize(width * height);
    y_lut.resize(width * height);
    z_lut.resize(width * height);

    X = std::make_unique<Eigen::Matrix <float, Eigen::Dynamic, 1>>(width * height);
    Y = std::make_unique<Eigen::Matrix <float, Eigen::Dynamic, 1>>(width * height);
    Z = std::make_unique<Eigen::Matrix <float, Eigen::Dynamic, 1>>(width * height);

    for (int i = 0; i < width; i++) {
        float angle_0 = 2.0 * PI * i / width;
        for (int j = 0; j < height; j++) {
            float angle = (beam_azim_angles[j] * PI / 180.0f) + angle_0;
            x_lut[i*height + j] = std::cos(beam_alt_angles[j] * PI / 180.0f) * std::cos(angle);
            y_lut[i*height + j] = -std::cos(beam_alt_angles[j] * PI / 180.0f) * std::sin(angle);
            z_lut[i*height + j] = std::sin(beam_alt_angles[j] * PI / 180.0f);
        }
    }
}

void LidarDriver::handle_lidar() {
    // std::cout << "Handle Lidar Start\n";
    // mutex lock
	//#pragma omp parallel for ordered num_threads(2)
    for(int i = 0; i <  ouster::OS1::columns_per_buffer; i++) {
	    const uint8_t* col_buf = ouster::OS1::nth_col(i, lidar_buf);
	    if(ouster::OS1::col_valid(col_buf)) {
	        times_buffer[counter*ouster::OS1::columns_per_buffer + i] = ouster::OS1::col_timestamp(col_buf);
	        const uint8_t* px;
	        for(int j = 0; j < height; j++) {
	            px = ouster::OS1::nth_px(j+height, col_buf);
	            ranges_buffer[counter*ouster::OS1::columns_per_buffer*height + i*height + j] = ouster::OS1::px_range(px);
	            intensities_buffer[counter*ouster::OS1::columns_per_buffer*height + i*height + j] = ouster::OS1::px_reflectivity(px);
	        }
	    }
	}
    // mutex unlock
	counter++;
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
    scan_counter++;
    // std::cout << "Handle Lidar Scan start\n";
    for(int i = 0; i < width; i++) {
        for(uint8_t j = 0; j < height; j++) {
            (*X)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * x_lut[i*height + j];
            (*Y)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * y_lut[i*height + j];
            (*Z)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * z_lut[i*height + j];
        }
    }
    // std::ofstream file("cart.txt");
    // if (file.is_open())
    //     file << cart;
    // file.close();
    // std::cout << "Handle Lidar Scan end\n";
}
