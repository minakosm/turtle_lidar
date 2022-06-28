#include <eigen3/Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "PointCloudProcessing/PoincloudProcessing.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ouster/client.h"
#include "ouster/types.h"
#include "ouster/impl/parsing.h"

#ifndef PI
#define PI 3.141593
#endif

class OusterDriver : public rclcpp::Node{
    private:
        
        ouster::sensor::lidar_mode lidarMode;
        ouster::sensor::timestamp_mode timestampMode;
        std::string lidar_ip, host_ip;

        int height;
        int width;
        int rotationRate;
        int packetsPerScan;
        float lidar_origin_to_beam_origin;

        uint8_t counter;
        int scan_counter;

        std::shared_ptr<ouster::sensor::client> cli;
        ouster::sensor::sensor_info lidarInfo;
        uint8_t* lidar_buf;
        uint8_t* imu_buf;

        std::vector<float> beam_azim_angles;
        std::vector<float> beam_alt_angles;

        std::vector<uint64_t> times_buffer;
        std::vector<uint32_t> ranges_buffer;
        std::vector<uint16_t> intensities_buffer;

        std::vector<uint64_t> times_process_buffer;
        std::vector<uint32_t> ranges_process_buffer;
        std::vector<uint16_t> intensities_process_buffer;

        // Lookup tables (LUT) for finding XYZ coordinates faster using pixel range
        Eigen::Array<float, Eigen::Dynamic, 3> directionLut;
        Eigen::Array<float, Eigen::Dynamic, 3> offsetLut;

        //Variables used in pointcloud processor
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 1>> X, Y, Z;
        std::unique_ptr<Eigen::Matrix <uint16_t, Eigen::Dynamic, 1>> intensities;
        Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos;

        //Variables related to publishing raw pointcloud recieved from LiDAR
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rawPointcloudPublisher;
        sensor_msgs::msg::PointCloud2 rawPointcloudMsg;
        bool publishRaw;

        bool runPipeline;
        bool invertXY;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredPointcloudPublisher;
        sensor_msgs::msg::PointCloud2 filteredPointcloudMsg;
        bool publishFilteredPcl;

    
        std::mutex pclProcessorMutex;
        int maxPointsProcessing, timeoutProcessing;
        PointcloudProcessing pointcloudProcessor;


    public:
        OusterDriver(std::string configFilePath = "./lidarConfig.ini");
        ~OusterDriver();

        void readSettingsFromINI(std::string pathToIniFile);
        int runDriver();
        void initialize();
        void initializePublishers();
        void makeXYZLut();
        void handleLidar();
        void handleLidarScan();
        void publishRawPointcloud();   
        void publishFilteredPointcloud(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> filteredPoints, Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> intensitiesFiltered);
};