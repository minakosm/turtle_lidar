
#include <iostream>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Eigen>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <chrono>
#include <cstdint>

#include "ousterDriver.h"
#include "PointCloudProcessing/PoincloudProcessing.h"

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
#include "ouster/impl/parsing.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#ifndef PI
#define PI 3.141593
#endif

OusterDriver::OusterDriver(std::string configFilePath) : Node("OusterDriver"),
                                                         pointcloudProcessor(configFilePath){

    readSettingsFromINI(configFilePath);
    
    height = 32;
    counter = 0;
    scan_counter = 0;
    switch(lidarMode){
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
            RCLCPP_ERROR(this->get_logger(), "Invaid lidar_mode");
            return;
    } 
    times_buffer.resize(width);
    ranges_buffer.resize(width * height);
    intensities_buffer.resize(width * height);
    
    pointcloudProcessor.resizeCoordinates(width * height);

    times_process_buffer.resize(width);
    ranges_process_buffer.resize(width * height);
    intensities_process_buffer.resize(width * height);

    X = std::make_unique<Eigen::Matrix<float, Eigen::Dynamic, 1>>(width * height);
    Y = std::make_unique<Eigen::Matrix<float, Eigen::Dynamic, 1>>(width * height);
    Z = std::make_unique<Eigen::Matrix<float, Eigen::Dynamic, 1>>(width * height);
    intensities = std::make_unique<Eigen::Matrix <uint16_t, Eigen::Dynamic, 1>>(width * height);

    runDriver();
}

OusterDriver::~OusterDriver(){
    delete[] imu_buf;
    delete[] lidar_buf;
}

void OusterDriver::readSettingsFromINI(std::string pathToIniFile){
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(pathToIniFile, pt);

    host_ip = pt.get<std::string>("Lidar.hostIP");
    lidar_ip = pt.get<std::string>("Lidar.lidarIP");
    lidarMode = (ouster::sensor::lidar_mode)pt.get<int>("Lidar.lidarMode");
    timestampMode = (ouster::sensor::timestamp_mode)pt.get<int>("Lidar.timestampMode");
    publishRaw = pt.get<bool>("Lidar.publishRawPointcloud");
    runPipeline = pt.get<bool>("Lidar.runPipeline");
    publishCones = pt.get<bool>("Lidar.publishConesDetectedPointcloud");
    lidar_origin_to_beam_origin = pt.get<float>("Lidar.lidar_origin_to_beam_origin");
    maxPointsProcessing = pt.get<int>("Lidar.maxPointsProcessing");
    timeoutProcessing = pt.get<int>("Lidar.timeoutProcessing");

    publishFilteredPcl = pt.get<bool>("Lidar.publishFilteredPcl");
}

int OusterDriver::runDriver(){
    cli = ouster::sensor::init_client(lidar_ip, host_ip, lidarMode, timestampMode);
    if (!cli){
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to client at: %s", lidar_ip);
        return 1;
    }
    RCLCPP_INFO(this->get_logger(), "Lidar Driver Initializing");
    initialize();
    initializePublishers();
    RCLCPP_INFO(this->get_logger(),"Lidar Driver Initialized");

    imu_buf = new uint8_t[ouster::sensor::impl::imu_packet_size + 1];
    lidar_buf = new uint8_t[ouster::sensor::lidar_packet_bytes_OS1_32 + 1];

    ouster::sensor::client_state st;
    while (rclcpp::ok()){
        st = ouster::sensor::poll_client(*cli, 1);
        if (st& ouster::sensor::CLIENT_ERROR){
            RCLCPP_ERROR(this->get_logger(), "Lidar Returned error status");
            return 3; 
        }
        else if (st & ouster::sensor::LIDAR_DATA){
            if (counter == 0){
                rawPointcloudMsg.header.stamp = rclcpp::Node::now();
            }
            if (ouster::sensor::read_lidar_packet(*cli, lidar_buf, ouster::sensor::lidar_packet_bytes_OS1_32)){
                handleLidar(); 
            }
            else 
                RCLCPP_ERROR(this->get_logger(),"read_lidar_packet failed");
        }
    }
    return -1;
}

void OusterDriver::initialize(){
    beam_azim_angles.resize(height);
    beam_alt_angles.resize(height);
    for (int i = 0; i < height; i++){
        beam_azim_angles[i] = cli->meta["beam_azimuth_angles"][i].asFloat();
        beam_alt_angles[i] = cli->meta["beam_altitude_angles"][i].asFloat();
    }
    makeXYZLut(); 
}

// TO BE MODIFIED. ONLY IF STATEMENT IS publishRaw
void OusterDriver::initializePublishers(){
    rclcpp::SensorDataQoS sensorQos;

    if(publishRaw){

        rawPointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ouster/rawPointcloud", sensorQos);

        rawPointcloudMsg.header.frame_id = "os1";

        rawPointcloudMsg.width = width;
        rawPointcloudMsg.height = height;
        rawPointcloudMsg.is_bigendian = false;
        rawPointcloudMsg.point_step = 18;
        rawPointcloudMsg.row_step = width * rawPointcloudMsg.point_step;
        rawPointcloudMsg.is_dense = true;

        rawPointcloudMsg.fields.resize(5); //x, y, z, t, intensity
        
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

        rawPointcloudMsg.data.resize(rawPointcloudMsg.point_step * width * height);
    }
    if(publishCones) {
        conesDetectedPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_landmarks", sensorQos);

        conesDetectedMsg.header.frame_id = "os1";

        conesDetectedMsg.is_bigendian = false;
        conesDetectedMsg.is_dense = true;
        conesDetectedMsg.point_step = 12;

        conesDetectedMsg.fields.resize(3);

        conesDetectedMsg.fields[0].name = "x";
        conesDetectedMsg.fields[0].offset = 0;
        conesDetectedMsg.fields[0].datatype = 7;
        conesDetectedMsg.fields[0].count = 1;

        conesDetectedMsg.fields[1].name = "y";
        conesDetectedMsg.fields[1].offset = 4;
        conesDetectedMsg.fields[1].datatype = 7;
        conesDetectedMsg.fields[1].count = 1;

        conesDetectedMsg.fields[2].name = "z";
        conesDetectedMsg.fields[2].offset = 8;
        conesDetectedMsg.fields[2].datatype = 7;
        conesDetectedMsg.fields[2].count = 1;
    }
    if(publishFilteredPcl){
        //DEBUG1
        RCLCPP_INFO(this->get_logger(),"debug1");
        filteredPointcloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ouster/filteredPointcloud", sensorQos);

        filteredPointcloudMsg.header.frame_id = "os1";

        filteredPointcloudMsg.height = 1;
        
        filteredPointcloudMsg.is_bigendian = false;
        filteredPointcloudMsg.is_dense = true;
        filteredPointcloudMsg.point_step = 18;


        filteredPointcloudMsg.fields.resize(5); //x, y, z, t, intensity
        
        filteredPointcloudMsg.fields[0].name = "x";
        filteredPointcloudMsg.fields[0].offset = 0;
        filteredPointcloudMsg.fields[0].datatype = 7;
        filteredPointcloudMsg.fields[0].count = 1;

        filteredPointcloudMsg.fields[1].name = "y";
        filteredPointcloudMsg.fields[1].offset = 4;
        filteredPointcloudMsg.fields[1].datatype = 7;
        filteredPointcloudMsg.fields[1].count = 1;

        filteredPointcloudMsg.fields[2].name = "z";
        filteredPointcloudMsg.fields[2].offset = 8;
        filteredPointcloudMsg.fields[2].datatype = 7;
        filteredPointcloudMsg.fields[2].count = 1;

        filteredPointcloudMsg.fields[3].name = "t";
        filteredPointcloudMsg.fields[3].offset = 12;
        filteredPointcloudMsg.fields[3].datatype = 6;
        filteredPointcloudMsg.fields[3].count = 1;

        filteredPointcloudMsg.fields[4].name = "intensity";
        filteredPointcloudMsg.fields[4].offset = 16;
        filteredPointcloudMsg.fields[4].datatype = 4;
        filteredPointcloudMsg.fields[4].count = 1;
    }

}

void OusterDriver::makeXYZLut(){

    Eigen::ArrayXf encoder(width * height);
    Eigen::ArrayXf azimuth(width * height);
    Eigen::ArrayXf altitude(width * height);

    const float azimuth_radians = PI * 2.0 / width ;

    for (int i = 0 ; i < width; i++){
        for (int j = 0; j < height; j++){
            encoder(i * height + j) = 2 * PI - (i * azimuth_radians);
            azimuth(i * height + j) = -beam_azim_angles[j] * PI / 180.0f;
            altitude(i * height + j) = beam_alt_angles[j] * PI / 180.0f;
        }
    }

    directionLut.resize(width * height, 3);
    offsetLut.resize(width * height, 3);

    //unit vectors for each pixel
    directionLut.col(0) = (encoder + azimuth ).cos() - altitude.cos();
    directionLut.col(1) = (encoder + azimuth).sin() - altitude.cos();
    directionLut.col(2) = altitude.sin();

    //offsets due to beam origin
    offsetLut.col(0) = encoder.cos() - directionLut.col(0);
    offsetLut.col(1) = encoder.sin() - directionLut.col(1);
    offsetLut.col(2) = -directionLut.col(2);
    offsetLut *= lidar_origin_to_beam_origin;
}

void OusterDriver::handleLidar(){
    const uint8_t* measurement_block_buf_check = ouster::sensor::impl::nth_measurement_block<32>(0, lidar_buf);
    //if(ouster::sensor::impl::measurement_block_frame_id(measurement_block_buf_check) < 20 || (counter == 0 && ouster::sensor::impl::measurement_block_measurement_id(measurement_block_buf_check) !=0))
    //    return;

    for(int i = 0; i < ouster::sensor::impl::measurement_blocks_per_packet; i++){
        const uint8_t* measurement_block_buf = ouster::sensor::impl::nth_measurement_block<32>(i, lidar_buf);

        if(ouster::sensor::impl::measurement_block_status<32>(measurement_block_buf)){
            times_buffer[counter * ouster::sensor::impl::measurement_blocks_per_packet + i] = ouster::sensor::impl::measurement_block_timestamp(measurement_block_buf);
            const uint8_t* px;
            for(int j = 0; j < height; j++){
                px = ouster::sensor::impl::nth_channel(j, measurement_block_buf);
                ranges_buffer[counter*ouster::sensor::impl::measurement_blocks_per_packet*height + i*height + j] = ouster::sensor::impl::channel_range(px);
                intensities_buffer[counter*ouster::sensor::impl::measurement_blocks_per_packet*height + i*height + j] = ouster::sensor::impl::channel_reflectivity(px);

            }
        }
    }
    counter++;
    if (counter == OusterDriver::packetsPerScan){
        counter = 0;
        times_buffer.swap(times_process_buffer);
        ranges_buffer.swap(ranges_process_buffer);
        intensities_buffer.swap(intensities_process_buffer);

        std::thread t(&OusterDriver::handleLidarScan, this);
        t.detach();
    }
}

void OusterDriver::handleLidarScan(){
    scan_counter++;
    for(int i = 0; i< width * height; i++){
        (*X)(i) = ranges_process_buffer[i] * 0.001 * directionLut(i, 0);

        (*Y)(i) = ranges_process_buffer[i] * 0.001 * directionLut(i, 1);

        (*Z)(i) = ranges_process_buffer[i] * 0.001 * directionLut(i, 2);

        (*intensities)(i) = intensities_process_buffer[i];
    }

    if(publishRaw)
        publishRawPointcloud();
    
    if(runPipeline) {
        if(pclProcessorMutex.try_lock()) {
            // RCLCPP_INFO(this->get_logger(), "PCL Processor started");
            int processReturnFlag = pointcloudProcessor.pipeline(X, Y, Z, intensities, conePos, maxPointsProcessing, timeoutProcessing);
            if(processReturnFlag == 0) {
                RCLCPP_INFO(this->get_logger(), "Filtered ground PCL size = %d\n", pointcloudProcessor.getNonGroundPoints());
                RCLCPP_INFO(this->get_logger(), "Found %u cones", conePos.rows());
                if(publishCones)
                    publishDetectedCones(0.0, 0.0, 0.0);
                if(publishFilteredPcl)
                    publishFilteredPointcloud(pointcloudProcessor.getCart(), pointcloudProcessor.getIntensitiesFiltered());
            }
            else if(processReturnFlag == -100) {
                RCLCPP_INFO(this->get_logger(), "PCL Processor timed-out");
                RCLCPP_INFO(this->get_logger(), "Filtered ground PCL size = %d\n", pointcloudProcessor.getNonGroundPoints());
            }
            else if(processReturnFlag == -101) {
                RCLCPP_INFO(this->get_logger(), "PCL Processor point limit reached");
                RCLCPP_INFO(this->get_logger(), "Filtered ground PCL size = %d\n", pointcloudProcessor.getNonGroundPoints());
            }
            else {
                RCLCPP_INFO(this->get_logger(), "PCL Processor returned ERROR flag: %d", processReturnFlag);
            }
            // RCLCPP_INFO(this->get_logger(), "PCL Processor ended");
            pclProcessorMutex.unlock();
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Pipeline mutex is locked, ignoring current pointcloud\n");
        }
    }

}


void OusterDriver::publishDetectedCones(double xPos, double yPos, double yaw){
    Eigen::Matrix<float, Eigen::Dynamic, 1> xCones(conePos.rows()), yCones(conePos.rows());
    xCones = (conePos.col(0).array() * std::cos(yaw) - conePos.col(1).array()*std::sin(yaw) + xPos).matrix();
    yCones = (conePos.col(0).array() * std::sin(yaw) - conePos.col(1).array()*std::cos(yaw) + yPos).matrix();

    conesDetectedMsg.width = conePos.rows();
    conesDetectedMsg.height = 1;
    conesDetectedMsg.row_step = conesDetectedMsg.width * conesDetectedMsg.point_step;
    conesDetectedMsg.data.resize(conesDetectedMsg.row_step);
    uint8_t* ptr = conesDetectedMsg.data.data();
    for(int i = 0; i < xCones.size(); i++){
        *((float*)(ptr + i*conesDetectedMsg.point_step)) = xCones(i);
        *((float*)(ptr + i*conesDetectedMsg.point_step + 4)) = yCones(i);
        *((float*)(ptr+ i*conesDetectedMsg.point_step + 8)) = 0.0; 
    } 
    conesDetectedPublisher->publish(conesDetectedMsg);
}


void OusterDriver::publishRawPointcloud(){
    uint8_t* ptr = rawPointcloudMsg.data.data();
    rawPointcloudMsg.header.stamp = rclcpp::Node::now();
    for (int i = 0; i < X->size(); i++){
        *((float*)(ptr + i*rawPointcloudMsg.point_step)) = (*X)(i);
        *((float*)(ptr + i*rawPointcloudMsg.point_step + 4)) = (*Y)(i);
        *((float*)(ptr + i*rawPointcloudMsg.point_step + 8)) = (*Z)(i);
        *((uint32_t*)(ptr + i*rawPointcloudMsg.point_step + 12)) = (uint32_t)(times_process_buffer[i / height] / 1e+6);
        *((uint16_t*)(ptr + i*rawPointcloudMsg.point_step + 16)) = intensities_process_buffer[i];
    }
    rawPointcloudPublisher->publish(rawPointcloudMsg);
}

void OusterDriver::publishFilteredPointcloud(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> filteredPoints, Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> intensitiesFiltered){
    filteredPointcloudMsg.width = filteredPoints.rows();
    filteredPointcloudMsg.row_step = filteredPointcloudMsg.width * filteredPointcloudMsg.point_step;

    filteredPointcloudMsg.data.resize(filteredPointcloudMsg.row_step);

    uint8_t* ptr = filteredPointcloudMsg.data.data();
    filteredPointcloudMsg.header.stamp = rclcpp::Node::now();

    for(int i = 0; i < filteredPoints.rows(); i++){
        *((float*)(ptr +i*filteredPointcloudMsg.point_step)) = (filteredPoints)(i, 0);
        *((float*)(ptr +i*filteredPointcloudMsg.point_step + 4)) = (filteredPoints)(i, 1);
        *((float*)(ptr +i*filteredPointcloudMsg.point_step + 8)) = (filteredPoints)(i, 2);
        *((uint32_t*)(ptr + i*filteredPointcloudMsg.point_step + 12)) = (uint32_t)(times_process_buffer[i / height] / 1e+6);
        *((uint16_t*)(ptr + i*filteredPointcloudMsg.point_step + 16)) = intensitiesFiltered(i);
    }
    filteredPointcloudPublisher->publish(filteredPointcloudMsg);
}


