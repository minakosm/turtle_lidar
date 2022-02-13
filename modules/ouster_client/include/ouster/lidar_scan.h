/**
 * @file 
 * @brief Holds lidar data by field in row-major order.
 */

#pragma once

#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include "ouster/types.h"

namespace ouster{
/**
 * @brief Datastructure for efficient operations on aggregated lidar data.
 * 
 * Stores each field (range, intensity, etc.) contigously as a H x W block of
 * 4-byte unsigned integers, where H is the number of beams and W is the horizontal resolution (e.g. 512, 1024, 2048).
 * 
 * Note: this is the "staggered" representation where each colymn correstponds to a single measurement in time. Use the destagger() function to create an
 * image where columns correspond to a single azimuth angle.
 */

class LidarScan{
    public:
        static constexpr int N_FIELDS = 4;

        using raw_t = uint32_t;
        using ts_t = std::chrono::nanoseconds;
        using data_t = Eigen::Array<raw_t, Eigen::Dynamic, 3>;

        using DynStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;

        //XYZ coordinates with dimensions arranged contiguously in columns
        using Points = Eigen::Array<double, Eigen::Dynamic, 3>;

        //Data fields reported per channel
        enum Field {RANGE, INTENSITY, AMBIENT, REFLECTIVITY};

        //Measurement block information, other than the channel data
        struct BlockHeader{
            ts_t timestamp;
            uint32_t encoder;
            uint32_t status;
        };

        //Members variables: use with caution, some of these will become private.
        std::ptrdiff_t w{0};
        std::ptrdiff_t h{0};
        data_t data;
        std::vector<BlockHeader> headers{};
        int32_t frame_id{-1};

        //The deafult constructor creates an invalid 0x0 scan
        LidarScan() = default;

        /**
         * @brief Initialize an empty scan with the given horizontal / vertical resolution.
         * 
         * @param w horizontal resolution, i.e. the number of measurements per scan
         * @param h vertical rezolution, i.e. the number of channels
         */
        LidarScan(size_t w, size_t h)
            : w{static_cast<std::ptrdiff_t>(w)},
              h{static_cast<std::ptrdiff_t>(h)},
              data{w * h, N_FIELDS},
              headers{w, BlockHeader{ts_t{0}, 0, 0}} {};

        /**
         * @brief Access timestamps as vector
         * 
         * @returns copy of the measurement timestamps as a vector
         */

        std::vector<LidarScan::ts_t> timestamps() const{
            std::vector<LidarScan::ts_t> res;
            res.reserve(headers.size());
            for (const auto& h : headers) res.push_back(h.timestamp);
            return res;
        }

        /**
         * @brief Access measurement block header fields 
         * 
         * @return the header values for the specified measurement id 
         */

        BlockHeader& header(size_t m_id) {return headers.at(m_id);}

        /** @copydoc header(size_t m_id) */
        const BlockHeader& header(size_t m_id) const {return headers.at(m_id);}

        /**
         * @brief Access measurement block data
         * 
         * @param m_id the measurement id of the desired block
         * @return a view of the measurement block data
         */
        Eigen::Map<data_t, Eigen::Unaligned, DynStride> block(size_t m_id) {
            return Eigen::Map<data_t, Eigen::Unaligned, DynStride>(data.row(m_id).data(), h, N_FIELDS, {w * h, w});
        }

        /** @copydoc block(size_t m_id) */
        Eigen::Map<const data_t, Eigen::Unaligned, DynStride> block(size_t m_id) const {
            return Eigen::Map<const data_t, Eigen::Unaligned, DynStride>(data.row(m_id).data(), h, N_FIELDS, {w * h, w});
    }


        /**
         * @brief Access a lidar data field
         * 
         * @param f the field to view
         * @return a view of the field data
         */
        Eigen::Map<img_t<raw_t>> field(Field f) {
            return Eigen::Map<img_t<raw_t>>(data.col(f).data(), h, w);
        }

        /** @copydoc field(Field f) */
        Eigen::Map<const img_t<raw_t>> field(Field f) const {
        return Eigen::Map<const img_t<raw_t>>(data.col(f).data(), h, w);
    }

};

//Equality for column headers 
inline bool operator==(const LidarScan::BlockHeader& a, const LidarScan::BlockHeader& b) {
return a.timestamp == b.timestamp && a.encoder == b.encoder && a.status == b.status;
}

//Equality for scans
inline bool operator==(const LidarScan& a, const LidarScan& b){
    return a.w == b.w && a.h == b.h && (a.data == b.data).all() &&
           a.headers == b.headers && a.frame_id && b.frame_id;
}

//LUT of beam directions and offsets
struct XYZLut {
    LidarScan::Points direction;
    LidarScan::Points offset;
};

/**
 * @brief Generate a matrix of unit vectors pointing radially outwards
 * 
 * Useful for efficiently computing cartesian coordinates from ranges. The result is a n x 3 array of 
 * doubles stored in column-major order where each row is the unit vector corresponding to the 
 * nth point in a lidar scan, with 0 <= n < h * w
 * 
 * @param w number of columns in the lidar scan. e.g. 512, 1024, 2048
 * @param h number of rows in the lidar scan 
 * @param range_unit the unit, in meters, of the range, e.g. sensor::range_unit
 * @param lidar_origin_to_beam_origin_mm the radius to the beam origin point of the unit, in millimeters
 * @param transform additional transformation to apply to resulting points 
 * @param azimuth_angles_deg azimuth offsets in degrees for each of the h beams
 * @param altitude_angles_deg altitude in degrees for each of the h beams 
 * 
 * @return xyz direction unit vectors for each point in the lidar scan
 */
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg);

/**
 * @brief Convenient overload that uses parameters from the supplied sensor_info
 * 
 * @param senso metadata returned from the client
 * @return xyz direction unit vectors for each point in the lidar scan
 */
inline XYZLut make_xyz_lut(const sensor::sensor_info& sensor){
    return make_xyz_lut(sensor.format.columns_per_frame, sensor.format.pixels_per_column,
                        sensor::range_unit, sensor.lidar_origin_to_beam_origin_mm,
                        sensor.lidar_to_sensor_transform, sensor.beam_azimuth_angles,
                        sensor.beam_altitude_angles);
}

/**
 * @brief Convert LidarScan to cartesian points
 * 
 * @param scan a LidarScan
 * @param xyz_lut a lookup table of unit vectors generated by make_xyz_lut
 * @return cartesian points where i-th row is a 3D point which corresponds to i-ith pixel
 * in LidarScan where i = row * w + col 
 */
LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut);

class ScanBatcher {
    std::ptrdiff_t w;
    std::ptrdiff_t h;
    uint16_t next_m_id ;
    LidarScan ls_write;

    public:
        sensor::packet_format pf;

        /**
         * @brief Create a batcher given information about the scan and packet format.
         * 
         * @param w number of columns in the lidar scan. One of 512, 1024, or 2048
         * @param pf expected format of the incoming packets used for parsing
         */
        ScanBatcher(size_t w, const sensor::packet_format& pf);

        /**
         * @brief Add a packet to the scan.
         * 
         * @param packet_buf the lidar packet 
         * @param lidar scan to populate
         * @return true when the provided lidar scan is ready to use
         */
        bool operator()(const uint8_t* packet_buf, LidarScan& ls);
};

}  //namespace ouster