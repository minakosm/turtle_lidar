#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <json/json.h>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <chrono>
#include <cstdint>

#include "OusterDriver.h"
#include "PointcloudProcessing.h"

#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster/impl/parsing.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#ifndef PI
#define PI 3.141593
#endif

OusterDriver::OusterDriver() : Node("OusterDriver"), pointcloudProcessor() {
    readSettingsFromINI();
    height = 32;
    counter = 0;
    scan_counter = 0;
    switch(lidarMode) {
        case ouster::sensor::MODE_512x10:
            width = 512;
            rotationRate = 10;
            packetsPerScan = 32;
            break;
        case ouster::sensor::MODE_512x20:
            width = 512;
            rotationRate = 20;
            packetsPerScan = 32;
            break;
        case ouster::sensor::MODE_1024x10:
            width = 1024;
            rotationRate = 10;
            packetsPerScan = 64;
            break;
        case ouster::sensor::MODE_1024x20:
            width = 1024;
            rotationRate = 20;
            packetsPerScan = 64;
            break;
        case ouster::sensor::MODE_2048x10:
            width = 2048;
            rotationRate = 10;
            packetsPerScan = 128;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Invalid lidar_mode");
            return;
    }
    times_buffer.resize(width);
    ranges_buffer.resize(width * height);
    intensities_buffer.resize(width * height);

    times_process_buffer.resize(width);
    ranges_process_buffer.resize(width * height);
    intensities_process_buffer.resize(width * height);

    runDriver();
}

OusterDriver::~OusterDriver() {
    delete[] lidar_buf;
}

void OusterDriver::readSettingsFromINI(std::string pathToIniFile) {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(pathToIniFile, pt);

    host_ip = pt.get<std::string>("Lidar.hostIP");
    lidar_ip = pt.get<std::string>("Lidar.lidarIP");
    lidarMode = (ouster::sensor::lidar_mode)pt.get<int>("Lidar.lidarMode");
    timestampMode = (ouster::sensor::timestamp_mode)pt.get<int>("Lidar.timestampMode");
    publishRaw = pt.get<bool>("Lidar.publishRawPointcloud");
}

int OusterDriver::runDriver() {
    cli = ouster::sensor::init_client(lidar_ip, host_ip, lidarMode, timestampMode);
    if (!cli) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to client at: %s", lidar_ip);
        return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Lidar Driver Initializing");
    initialize();
    RCLCPP_INFO(this->get_logger(), "Lidar Driver Initialized");

    lidar_buf = new uint8_t[ouster::sensor::lidar_packet_bytes_OS1_32 + 1];

    ouster::sensor::client_state st;
    while (rclcpp::ok()) {
        st = ouster::sensor::poll_client(*cli, 1);
        if (st & ouster::sensor::CLIENT_ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Lidar returned error status");
            return 3;
        }
        else if (st & ouster::sensor::LIDAR_DATA) {
            if (ouster::sensor::read_lidar_packet(*cli, lidar_buf, ouster::sensor::lidar_packet_bytes_OS1_32)) {
                handleLidar();
            }
            else
                RCLCPP_ERROR(this->get_logger(), "read_lidar_packet failed");
        }
    }
    return -1;
}

void OusterDriver::initialize() {
    beam_azim_angles.resize(height);
    beam_alt_angles.resize(height);
    for(int i = 0; i < height; i++) {
        beam_azim_angles[i] = cli->meta["beam_azimuth_angles"][i].asFloat();
        beam_alt_angles[i] = cli->meta["beam_altitude_angles"][i].asFloat();
        std::cout << "beam_azim_angles[" << i << "] = " << beam_azim_angles[i] << "   "
                  << "beam_alt_angles[" << i << "] = " << beam_alt_angles[i] << std::endl;
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
    initializePublishers();
}

void OusterDriver::initializePublishers() {
    if(publishRaw) {
        rawPointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("RawPointcloud", 10);

        rawPointcloudMsg.header.frame_id = "laser_sensor_frame";

        rawPointcloudMsg.width = width;
        rawPointcloudMsg.height = height;
        rawPointcloudMsg.is_bigendian = false;
        rawPointcloudMsg.is_dense = true;
        rawPointcloudMsg.point_step = 18;
        rawPointcloudMsg.row_step = width * rawPointcloudMsg.point_step;
        
        rawPointcloudMsg.fields.resize(5);
        
        rawPointcloudMsg.fields[0].name = "x";
        rawPointcloudMsg.fields[0].offset = 0;
        rawPointcloudMsg.fields[0].datatype = 7;
        rawPointcloudMsg.fields[0].count = 1;
        
        rawPointcloudMsg.fields[1].name = "y";
        rawPointcloudMsg.fields[1].offset = 4;
        rawPointcloudMsg.fields[1].datatype = 7;
        rawPointcloudMsg.fields[1].count = 1;
        
        rawPointcloudMsg.fields[2].name = "z";
        rawPointcloudMsg.fields[2].offset = 8;
        rawPointcloudMsg.fields[2].datatype = 7;
        rawPointcloudMsg.fields[2].count = 1;

        rawPointcloudMsg.fields[3].name = "t";
        rawPointcloudMsg.fields[3].offset = 12;
        rawPointcloudMsg.fields[3].datatype = 6;
        rawPointcloudMsg.fields[3].count = 1;

        rawPointcloudMsg.fields[4].name = "intensity";
        rawPointcloudMsg.fields[4].offset = 16;
        rawPointcloudMsg.fields[4].datatype = 4;
        rawPointcloudMsg.fields[4].count = 1;
        
        rawPointcloudMsg.data.resize(rawPointcloudMsg.point_step * X->size());
    }
}

void OusterDriver::handleLidar() {
    // RCLCPP_INFO(this->get_logger(), "Handle Lidar Start");
    // mutex lock
	//#pragma omp parallel for ordered num_threads(2)
    for(int i = 0; i < ouster::sensor::impl::cols_per_packet; i++) {
	    const uint8_t* col_buf = ouster::sensor::impl::nth_col<32>(i, lidar_buf);
	    if(ouster::sensor::impl::col_status<32>(col_buf)) {
	        times_buffer[counter * ouster::sensor::impl::cols_per_packet + i] = ouster::sensor::impl::col_timestamp(col_buf);
	        const uint8_t* px;
	        for(int j = 0; j < height; j++) {
	            px = ouster::sensor::impl::nth_px(j, col_buf);
	            ranges_buffer[counter*ouster::sensor::impl::cols_per_packet*height + i*height + j] = ouster::sensor::impl::px_range(px);
	            intensities_buffer[counter*ouster::sensor::impl::cols_per_packet*height + i*height + j] = ouster::sensor::impl::px_reflectivity(px);
	        }
	    }
	}
    // mutex unlock
	counter++;
	if(counter == packetsPerScan) {
	    counter = 0;
	    // mutex process buffers lock
	    times_buffer.swap(times_process_buffer);
	    ranges_buffer.swap(ranges_process_buffer);
	    intensities_buffer.swap(intensities_process_buffer);
        // mutex process buffers unlock

        std::thread t(&OusterDriver::handleLidarScan, this);
	    t.detach();
	}
    // RCLCPP_INFO(this->get_logger(), "Handle Lidar End");
}

void OusterDriver::handleLidarScan() {
    scan_counter++;
    // RCLCPP_INFO(this->get_logger(), "Handle Lidar Scan start");
    for(int i = 0; i < width; i++) {
        for(uint8_t j = 0; j < height; j++) {
            (*X)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * x_lut[i*height + j];
            (*Y)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * y_lut[i*height + j];
            (*Z)(i*height + j) = ranges_process_buffer[i*height + j] * 0.001 * z_lut[i*height + j];
        }
    }
    if(publishRaw)
        publishRawPointcloud();
    // RCLCPP_INFO(this->get_logger(), "Handle Lidar Scan end");
}

void OusterDriver::publishRawPointcloud() {
    uint8_t* ptr = rawPointcloudMsg.data.data();
    rawPointcloudMsg.header.stamp = rclcpp::Node::now();
    for(int i = 0; i < X->size(); i++) {
        *((float*)(ptr + i*rawPointcloudMsg.point_step)) = (*X)(i);
        *((float*)(ptr + i*rawPointcloudMsg.point_step + 4)) = (*Y)(i);
        *((float*)(ptr + i*rawPointcloudMsg.point_step + 8)) = (*Z)(i);
        *((uint32_t*)(ptr + i*rawPointcloudMsg.point_step + 12)) = (uint32_t)(times_process_buffer[i / height] / 1e+6);
        *((uint16_t*)(ptr + i*rawPointcloudMsg.point_step + 16)) = intensities_process_buffer[i];
    }
    rawPointcloudPublisher->publish(rawPointcloudMsg);
}
