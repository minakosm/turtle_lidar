#ifndef OUSTER_OFFLINE_H
#define OUSTER_OFFLINE_H

#include <eigen3/Eigen/Dense>
#include <json/json.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <vector>
#include <cstdint>
#include <memory>
#include <mutex>

#include "PointcloudProcessing.h"
#include "bayes.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#ifndef PI
#define PI 3.141593
#endif

class OusterOffline : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSubscriber;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclConePublisher, pclGroundFilteredPublisher;

        std::mutex pclProcessorMutex;
        PointcloudProcessing pclProcessor;

        std::unique_ptr<Eigen::VectorXf> X, Y, Z;
        Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> cartesianAfterFilterGround;
        std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensities;
        Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos;
        GNBC coneClassifier;

        sensor_msgs::msg::PointCloud2 conesDetectedMsg;
        sensor_msgs::msg::PointCloud2 pclAfterGroundFilterMsg;

        void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void subsPubsInit();
        void publishDetectedCones(double xPos, double yPos, double yaw);
        void publishGroundFilteredPCL(double xPos, double yPos, double yaw);

    public:
        OusterOffline(std::string configFilePath = "./lidarConfig.ini", 
                      std::string trainXFilePath = "./ousterConeTrainDataX.txt", 
                      std::string trainYFilePath = "./ousterConeTrainDataY.txt");
};

#endif // OUSTER_OFFLINE_H_INCLUDED