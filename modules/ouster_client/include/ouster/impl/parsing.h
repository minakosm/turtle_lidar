/**
 * @file 
 * @brief Packet parsing internals
 */

#pragma once

#include <cstdint>
#include <cstring>

#include "ouster/types.h"

namespace ouster{
namespace sensor{
namespace impl{


constexpr int header_block_size = 16; // Header block size 128 bits / 8 = 16 bytes
constexpr int channel_data_block_size = 12; //Channel data block size 94 bits / 8 = 12 bytes
constexpr int measurement_status_size = 4; // Measurement block status size 32 bit / 8 = 4 bytes

constexpr int imu_packet_size = 48; //Imu packet size : 12 words * 4 bytes = 48 bytes 

constexpr int measurement_blocks_per_packet = 16;

constexpr int64_t encoder_ticks_per_rev = 90112;

constexpr int measurement_block_bytes(int n_channels){
    return header_block_size + (n_channels * channel_data_block_size) + measurement_status_size;
}

constexpr int packet_bytes(int n_channels){
    return measurement_blocks_per_packet * measurement_block_bytes(n_channels);
}

// LIDAR PACKETS 

template <int N_CHANNELS>
const uint8_t* nth_measurement_block(int n, const uint8_t* lidar_buf){
    return lidar_buf + (n * measurement_block_bytes(N_CHANNELS));
}

template <int N_CHANNELS>
inline uint32_t measurement_block_status(const uint8_t* measurement_block_buf) {
    uint32_t res;
    std::memcpy(&res, measurement_block_buf + measurement_block_bytes(N_CHANNELS) - 4, sizeof(uint32_t));
    return res;
}

inline uint64_t measurement_block_timestamp(const uint8_t* measurement_block_buf){
    uint64_t res;
    std::memcpy(&res, measurement_block_buf, sizeof(uint64_t));
    return res; //nanoseconds
}

inline uint16_t measurement_block_measurement_id(const uint8_t* measurement_block_buf) {
    uint16_t res;
    std::memcpy(&res, measurement_block_buf + 8, sizeof(uint16_t));
    return res;
}

inline uint16_t measurement_block_frame_id(const uint8_t* measurement_block_buf){
    uint16_t res;
    std::memcpy(&res, measurement_block_buf + 10, sizeof(uint16_t));
    return res;
}

inline uint32_t measurement_block_encoder(const uint8_t* measurement_block_buf) {
    uint32_t res;
    std::memcpy(&res, measurement_block_buf + 12, sizeof(uint32_t));
    return res;
}

inline const uint8_t* nth_channel(int n, const uint8_t* measurement_block_buf){
    return measurement_block_buf + 16 + (n * channel_data_block_size);
}

inline uint32_t channel_range(const uint8_t* channel_buf){
    uint32_t res;
    std::memcpy(&res, channel_buf, sizeof(uint32_t));
    res &= 0x000fffff;
    return res;
}

inline uint16_t channel_reflectivity(const uint8_t* channel_buf){
    uint16_t res;
    std::memcpy(&res, channel_buf + 4, sizeof(uint16_t));
    return res;
}

inline uint16_t channel_signal(const uint8_t* channel_buf){
    uint16_t res;
    std::memcpy(&res, channel_buf + 6, sizeof(uint16_t));
    return res;
}

inline uint16_t channel_infrared(const uint8_t* channel_buf){
    uint16_t res;
    std::memcpy(&res, channel_buf + 8, sizeof(uint16_t));
    return res;
}

// IMU PACKETS 

inline uint64_t imu_sys_ts(const uint8_t* imu_buf){
    uint64_t res;
    std::memcpy(&res, imu_buf, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_accel_ts(const uint8_t* imu_buf){
    uint64_t res;
    std::memcpy(&res, imu_buf + 8, sizeof(uint64_t));
    return res;
}

inline uint64_t imu_gyro_ts(const uint8_t* imu_buf){
    uint64_t res;
    std::memcpy(&res, imu_buf + 16, sizeof(uint64_t));
    return res;
}

inline float imu_la_x(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 24, sizeof(float));
    return res;
}

inline float imu_la_y(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 28, sizeof(float));
    return res;
}

inline float imu_la_z(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 32, sizeof(float));
    return res;
}

inline float imu_av_x(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 36, sizeof(float));
    return res;
}

inline float imu_av_y(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 40, sizeof(float));
    return res;
}

inline float imu_av_z(const uint8_t* imu_buf){
    float res;
    std::memcpy(&res, imu_buf + 44, sizeof(float));
    return res;
}

template <int N_CHANNELS>
constexpr packet_format packet_2_0(){
    return {
        impl::packet_bytes(N_CHANNELS),
        impl::imu_packet_size,
        impl::measurement_blocks_per_packet,
        N_CHANNELS,
        impl::encoder_ticks_per_rev,

        impl::nth_measurement_block<N_CHANNELS>,
        impl::measurement_block_timestamp,
        impl::measurement_block_encoder,
        impl::measurement_block_measurement_id,
        impl::measurement_block_frame_id,
        impl::measurement_block_status<N_CHANNELS>,

        impl::nth_channel,
        impl::channel_range,
        impl::channel_reflectivity,
        impl::channel_signal,
        impl::channel_infrared,

        impl::imu_sys_ts,
        impl::imu_accel_ts,
        impl::imu_gyro_ts,
        impl::imu_la_x,
        impl::imu_la_y,
        impl::imu_la_z,
        impl::imu_av_x,
        impl::imu_av_y,
        impl::imu_av_z
    };
}

} // namespace impl
} // namespace sensor
} // namespace ouster