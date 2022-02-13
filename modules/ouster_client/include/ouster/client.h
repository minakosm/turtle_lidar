/**
 * @brief sample sensor client
 */

#pragma once 

#include <cstdint>
#include <memory>
#include <string>

#include <jsoncpp/json/json.h>

#include "ouster/types.h"
#include "ouster/version.h"
#include "ouster/impl/netcompat.h"

namespace ouster {
namespace sensor {

const int lidar_packet_bytes_OS1_32 = 6464;

struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    std::string hostname;
    Json::Value meta;
    ~client(){
        impl::socket_close(lidar_fd);
        impl::socket_close(imu_fd);
    }
};

enum client_state{
    TIMEOUT = 0,
    CLIENT_ERROR = 1,
    LIDAR_DATA = 2, 
    IMU_DATA = 4,
    EXIT = 8
};

/** Minimum supported version  */
const util::version min_version = {1, 12, 0};

//Listen for sensor data on the specific ports;
std::shared_ptr<client> init_client(const std::string& hostname="",
int lidar_port = 7502, int imu_port = 7503);

//Connect to and configure the sensor and start listening for data 
std::shared_ptr<client> init_client(const std::string& hostname,
const std::string& udp_dest_host,
lidar_mode mode = MODE_UNSPEC,
timestamp_mode ts_mode = TIME_FROM_UNSPEC,
int lidar_port = 0, int imu_port = 0,
int timeout_sec = 30);

//Block for up to timeout_sec until either data is ready or an error occured
client_state poll_client(const client& cli, int timeout_sec = 1);

//Read lidar data from the sensor.
bool read_lidar_packet(const client& cli, uint8_t* buf, const packet_format& pf);

bool read_lidar_packet(const client& cli, uint8_t* buf, const size_t len);

//Read imu data from the sensor  
bool read_imu_packet(const client& cli, uint8_t* buf, const packet_format& pf);

//Read imu data from the sensor
bool read_imu_packet(const client& cli, uint8_t* buf, const size_t len);

//Get metadata text blob from the sensor
std::string get_metadata(client& cli, int timeout_sec = 30);

}// namepsace sensor
}// namespace ouster