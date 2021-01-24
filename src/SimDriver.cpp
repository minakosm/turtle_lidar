#include <memory>
#include <eigen3/Eigen/Dense>
#include <mutex>

#include "SimDriver.h"
#include "rclcpp/qos.hpp"
#include "turtle_common/fsmath_conversions.hpp"
#include "utils.h"

SimDriver::SimDriver(std::string filepath, 
                     std::string trainXFilePath, 
                     std::string trainYFilePath) : Node("lidar_sim_driver"), 
                                                   pclProcessor(2048*10, filepath),
                                                   coneClassifier(),
                                                   conePos() {

    // Print used configuration and train files
    RCLCPP_INFO(this->get_logger(), "Simulation LiDAR Node is using the following paths:\nConfiguration INI file: %s\nCone classifier TrainX: %s\nCone classifier TrainY: %s\n\n", filepath.c_str(), trainXFilePath.c_str(), trainYFilePath.c_str());

    // Initialize unique_ptr(s)
    X = std::make_unique<Eigen::VectorXf>();
    Y = std::make_unique<Eigen::VectorXf>();
    Z = std::make_unique<Eigen::VectorXf>();
    intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();

    // Cone classifier training
    Eigen::MatrixXf coneTrainDataX;
    Eigen::Matrix<int, Eigen::Dynamic, 1> coneTrainDataY;
    read_matrix<float>(trainXFilePath, coneTrainDataX);
    read_vector<int>(trainYFilePath, coneTrainDataY);
    coneClassifier.train(coneTrainDataX, coneTrainDataY);

    subsPubsInit();

    RCLCPP_INFO(this->get_logger(), "Simulation LiDAR processing driver initiated");
}

void SimDriver::subsPubsInit() {
    conesDetectedMsg.header.frame_id = "odom";
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

    // conesDetectedMsg.fields[3].name = "intensity";
    // conesDetectedMsg.fields[3].offset = 12;
    // conesDetectedMsg.fields[3].datatype = 4;
    // conesDetectedMsg.fields[3].count = 1;

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    pclPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/conesDetected", 10);
    pclSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/pointcloud2_os1_bl_gen1", qos, std::bind(&SimDriver::pclCallback, this, std::placeholders::_1));
    odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SimDriver::odomCallback, this, std::placeholders::_1));
}

void SimDriver::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "PointCloud2 Message received START");
    auto a1 = std::chrono::steady_clock::now();

    double xPos, yPos, yaw;
    std::unique_lock<std::mutex> lock(odomMsgMutex);
    xPos = lastOdomMsg.pose.pose.position.x;
    yPos = lastOdomMsg.pose.pose.position.y;
    fsmath_conversions::fromQuanternionToYaw(lastOdomMsg.pose.pose.orientation.x,
                                             lastOdomMsg.pose.pose.orientation.y,
                                             lastOdomMsg.pose.pose.orientation.z,
                                             lastOdomMsg.pose.pose.orientation.w,
                                             yaw);
    lock.unlock();

    RCLCPP_INFO(this->get_logger(), "\nPointCloud2 size = %u\nx = %.2f   y = %.2f   yaw = %.4f", msg->width*msg->height, xPos, yPos, yaw);
    X->resize(msg->width*msg->height);
    Y->resize(msg->width*msg->height);
    Z->resize(msg->width*msg->height);
    intensities->resize(msg->width*msg->height);
    pclProcessor.resizeCoordinates(msg->width*msg->height);
    // RCLCPP_INFO(this->get_logger(), "PointCloud2 Message conversion to XYZ started");
    uint8_t* ptr = msg->data.data();
    for(int i = 0; i < X->size(); i++) {
        (*X)(i) = *((float*)(ptr + i*msg->point_step));
        (*Y)(i) = *((float*)(ptr + i*msg->point_step + 4));
        (*Z)(i) = *((float*)(ptr + i*msg->point_step + 8));
        (*intensities)(i) = (uint16_t)*((float*)(ptr + i*msg->point_step + 12));
    }
    // RCLCPP_INFO(this->get_logger(), "PointCloud2 Message converted to XYZ");

    if(pclProcessorMutex.try_lock()) {
        // RCLCPP_INFO(this->get_logger(), "PCL Processor started");
        int processReturnFlag = pclProcessor.pipeline(X, Y, Z, intensities, coneClassifier, conePos);
        if(processReturnFlag == 0) {
            RCLCPP_INFO(this->get_logger(), "Found %u cones", conePos.rows());
            publishDetectedCones(xPos, yPos, yaw);
        }
        else if(processReturnFlag == -100) {
            RCLCPP_INFO(this->get_logger(), "PCL Processor timed-out");
        }
        else if(processReturnFlag == -101) {
            RCLCPP_INFO(this->get_logger(), "PCL Processor point limit reached");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "PCL Processor returned ERROR flag: %d", processReturnFlag);
        }
        // RCLCPP_INFO(this->get_logger(), "PCL Processor ended");
        pclProcessorMutex.unlock();
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Pipeline mutex is locked, ignoring current pointcloud");
    }

    auto a2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Total pclCallback time in us: %lu", std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count());
    // RCLCPP_INFO(this->get_logger(), "PointCloud2 Message received END");
}

void SimDriver::publishDetectedCones(double xPos, double yPos, double yaw) {
    Eigen::Matrix<float, Eigen::Dynamic, 1> xCones(conePos.rows()), yCones(conePos.rows());
    xCones = (conePos.col(0).array()*std::cos(yaw) - conePos.col(1).array()*std::sin(yaw) + xPos).matrix();
    yCones = (conePos.col(0).array()*std::sin(yaw) + conePos.col(1).array()*std::cos(yaw) + yPos).matrix();

    conesDetectedMsg.header.stamp = rclcpp::Node::now();
    conesDetectedMsg.width = conePos.rows();
    conesDetectedMsg.height = 1;
    conesDetectedMsg.row_step = conesDetectedMsg.width * conesDetectedMsg.point_step;
    conesDetectedMsg.data.resize(conesDetectedMsg.row_step);
    uint8_t* ptr = conesDetectedMsg.data.data();
    for(int i = 0; i < xCones.size(); i++) {
        *((float*)(ptr + i*conesDetectedMsg.point_step)) = xCones(i);
        *((float*)(ptr + i*conesDetectedMsg.point_step + 4)) = yCones(i);
        *((float*)(ptr + i*conesDetectedMsg.point_step + 8)) = 0.0;
        // *((uint16_t*)(ptr + i*conesDetectedMsg.point_step + 12)) = 20;
        RCLCPP_INFO(this->get_logger(), "xCone = %f   yCone = %f", xCones(i), yCones(i));
    }
    pclPublisher->publish(conesDetectedMsg);
}

void SimDriver::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
    std::lock_guard<std::mutex> lock(odomMsgMutex);
    lastOdomMsg = *odomMsg;
}
