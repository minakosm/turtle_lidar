#include "ouster/lidar_scan.h"

#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <vector>

namespace ouster {

constexpr int LidarScan::N_FIELDS;

XYZLut make_xyz_lut(size_t w, size_t h, double range_unit,
                    double lidar_origin_to_beam_origin_mm,
                    const mat4d& transform,
                    const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg){
    Eigen::ArrayXd encoder(w * h); //theta_e
    Eigen::ArrayXd azimuth(w * h); //theta_a
    Eigen::ArrayXd altitude(w * h); //phi

    const double azimuth_radians = M_PI * 2.0 / w;

    // populate angles for each pixel
    for (size_t v = 0; v < w; v++){
        for (size_t u = 0; u < h; u++) {
            size_t i = u * w + v;
            encoder(i) = 2 * M_PI - (v * azimuth_radians);
            azimuth(i) = -azimuth_angles_deg[u] * M_PI / 180.0 ;
            altitude(i) = altitude_angles_deg[u] * M_PI / 180.0;
        }
    }

    XYZLut lut;

    //unit vectors for each pixel
    lut.direction = LidarScan::Points{w * h, 3};
    lut.direction.col(0) = (encoder + azimuth).cos() * altitude.cos();
    lut.direction.col(1) = (encoder + azimuth).sin() * altitude.cos();
    lut.direction.col(2) = altitude.sin();

    //offsets due to beam origin
    lut.offset = LidarScan::Points{w * h, 3};
    lut.offset.col(0) = encoder.cos() - lut.direction.col(0);
    lut.offset.col(1) = encoder.sin() - lut.direction.col(1);
    lut.offset.col(2) = -lut.direction.col(2);
    lut.offset *= lidar_origin_to_beam_origin_mm;

    // apply the supplied transform
    auto rot = transform.topLeftCorner(3, 3).transpose();
    auto trans = transform.topRightCorner(3, 1).transpose();
    lut.direction.matrix() *= rot;
    lut.offset.matrix() += trans.replicate(w * h, 1);

    //apply scaling factor
    lut.direction *= range_unit;
    lut.offset *= range_unit;

    return lut;
}

LidarScan::Points cartesian(const LidarScan& scan, const XYZLut& lut){
    auto reshaped = Eigen::Map<const Eigen::Array<LidarScan::raw_t, -1, 1>>(
        scan.field(LidarScan::RANGE).data(), scan.h * scan.w);
    auto nooffset = lut.direction.colwise() * reshaped.cast<double>();
    return (nooffset.array() == 0.0).select(nooffset, nooffset + lut.offset);
}

ScanBatcher::ScanBatcher(size_t w, const sensor::packet_format& pf)
    : w(w), h(pf.pixels_per_column), next_m_id(0), ls_write(w, h), pf(pf) {}


bool ScanBatcher::operator()(const uint8_t* packet_buf, LidarScan& ls) {
    using row_view_t =
        Eigen::Map<Eigen::Array<LidarScan::raw_t, Eigen::Dynamic,
                                Eigen::Dynamic, Eigen::RowMajor>>;

    if (ls.w != w || ls.h != h)
        throw std::invalid_argument("unexpected scan dimensions");

    bool swapped = false;

    for (int icol = 0; icol < pf.columns_per_packet; icol++) {
        const uint8_t* measurement_block_buf = pf.nth_measurement_block(icol, packet_buf);
        const uint16_t m_id = pf.measurement_block_measurement_id(measurement_block_buf);
        const uint16_t f_id = pf.measurement_block_frame_id(measurement_block_buf);
        const std::chrono::nanoseconds ts(pf.measurement_block_timestamp(measurement_block_buf));
        const uint32_t encoder = pf.measurement_block_encoder(measurement_block_buf);
        const uint32_t status = pf.measurement_block_status(measurement_block_buf);
        const bool valid = (status == 0xffffffff);

        // drop invalid / out-of-bounds data in case of misconfiguration
        if (!valid || m_id >= w || f_id + 1 == ls_write.frame_id) continue;

        if (ls_write.frame_id != f_id) {
            // if not initializing with first packet
            if (ls_write.frame_id != -1) {
                // zero out remaining missing columns
                auto rows = h * LidarScan::N_FIELDS;
                row_view_t{ls_write.data.data(), rows, w}
                    .block(0, next_m_id, rows, w - next_m_id)
                    .setZero();

                // finish the scan and notify callback
                std::swap(ls, ls_write);
                swapped = true;
            }

            // start new frame
            next_m_id = 0;
            ls_write.frame_id = f_id;
        }

        // zero out missing columns if we jumped forward
        if (m_id >= next_m_id) {
            auto rows = h * LidarScan::N_FIELDS;
            row_view_t{ls_write.data.data(), rows, w}
                .block(0, next_m_id, rows, m_id - next_m_id)
                .setZero();
            next_m_id = m_id + 1;
        }

        ls_write.header(m_id) = {ts, encoder, status};
        for (uint8_t ipx = 0; ipx < h; ipx++) {
            const uint8_t* channel_buf = pf.nth_channel(ipx, measurement_block_buf);

            ls_write.block(m_id).row(ipx)
                << static_cast<LidarScan::raw_t>(pf.channel_range(channel_buf)),
                static_cast<LidarScan::raw_t>(pf.channel_signal(channel_buf)),
                static_cast<LidarScan::raw_t>(pf.channel_infrared(channel_buf)),
                static_cast<LidarScan::raw_t>(pf.channel_reflectivity(channel_buf));
        }
    }
    return swapped;
}

}//namespace ouster