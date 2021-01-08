#ifndef SIM_DRIVER_H
#define SIM_DRIVER_H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <mutex>

#include "PointcloudProcessing.h"
#include "bayes.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"


#ifndef PI
#define PI 3.141593
#endif

class SimDriver : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pclSubscriber;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPublisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber;

        PointcloudProcessing pclProcessor;
        std::unique_ptr<Eigen::VectorXf> X, Y, Z;
        std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensities;
        Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos;
        GNBC coneClassifier;

        nav_msgs::msg::Odometry lastOdomMsg;
        sensor_msgs::msg::PointCloud2 conesDetectedMsg;

        std::mutex odomMsgMutex;


        void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

        void subsPubsInit();
        void publishDetectedCones(double xPos, double yPos, double yaw);

    public:
        SimDriver(std::string configFilePath = "./config.ini", 
                  std::string trainXFilePath = "./trainDataX.txt", 
                  std::string trainYFilePath = "./trainDataY.txt");
};

#endif // SIM_DRIVER_H_INCLUDED