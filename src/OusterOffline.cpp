#include "OusterOffline.h"

#include <memory>
#include <eigen3/Eigen/Dense>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

OusterOffline::OusterOffline(std::string filepath) : Node("ouster_offline"),
                                                     pclProcessor(2048*10, filepath),
                                                     filteredGround(),
                                                     conePos(){

// Print used Configuration
RCLCPP_INFO(this->get_logger(), "Offline Lidar Node is using the following paths: \nConfiguration INI file: %s", filepath.c_str());

// Initialize unique_ptr(s)
X = std::make_unique<Eigen::VectorXf>();
Y = std::make_unique<Eigen::VectorXf>();
Z = std::make_unique<Eigen::VectorXf>();
intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();

subsPubsInit();

RCLCPP_INFO(this->get_logger(), "Offline LiDAR Processing Driver Initiated");
}

void OusterOffline::subsPubsInit(){
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

    filteredGroundMsg.header.frame_id = "os1";
    filteredGroundMsg.is_bigendian = false;
    filteredGroundMsg.is_dense = true;
    filteredGroundMsg.point_step = 12;
    filteredGroundMsg.fields.resize(3);

    filteredGroundMsg.fields[0].name = "x";
    filteredGroundMsg.fields[0].offset = 0;
    filteredGroundMsg.fields[0].datatype = 7;
    filteredGroundMsg.fields[0].count = 1;

    filteredGroundMsg.fields[1].name = "y";
    filteredGroundMsg.fields[1].offset = 4;
    filteredGroundMsg.fields[1].datatype = 7;
    filteredGroundMsg.fields[1].count = 1;

    filteredGroundMsg.fields[2].name = "z";
    filteredGroundMsg.fields[2].offset = 8;
    filteredGroundMsg.fields[2].datatype = 7;
    filteredGroundMsg.fields[2].count = 1;

    // conesDetectedMsg.fields[3].name = "intensity";
    // conesDetectedMsg.fields[3].offset = 12;
    // conesDetectedMsg.fields[3].datatype = 4;
    // conesDetectedMsg.fields[3].count = 1;


    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    pclConePosPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/conesDetectedOffline", 10);
    pclGroundFilteredPublisher = this-> create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/groundFiltered", 10);
    pclSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("ouster/rawPointcloud", qos, std::bind(&OusterOffline::pclCallback, this, std::placeholders::_1));
}

void OusterOffline::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    auto a1 = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "\nPointCloud2 size = %u", msg->width * msg->height);
    
    X->resize(msg->width * msg->height);
    Y->resize(msg->width * msg->height);
    Z->resize(msg->width * msg->height);
    intensities->resize(msg->width * msg->height);
    pclProcessor.resizeCoordinates(msg->width * msg->height);

    // RCLCPP_INFO(this->get_logger(),"Initiate PointCloud2 Message converion to XYZ");
    uint8_t* ptr = msg->data.data();
    for(int i=0; i < X->size(); i++){
        (*X)(i) = *((float*)(ptr + i*msg->point_step));
        (*Y)(i) = *((float*)(ptr + i*msg->point_step + 4));
        (*Z)(i) = *((float*)(ptr + i*msg->point_step + 8));
        (*intensities)(i) = *((float*)(ptr + i*msg->point_step + 12));
    }
    // RCLCPP_INFO(this->get_logger(),"PointCloud2 Message converted to XYZ");

    if(pclProcessorMutex.try_lock()){
        // RCLCPP_INFO(this->get_logger(),"PCL Processor started");
        int processReturnFlag = pclProcessor.pipeline(X, Y, Z, intensities, conePos);
        switch (processReturnFlag)
        {
        case 0:
            RCLCPP_INFO(this->get_logger(),"Found %u cones", conePos.rows());
            publihsDetectedCones(0.0, 0.0, 0.0);
            break;
        case -100:
            RCLCPP_INFO(this->get_logger(),"PCL Processor Timed out");
            break;

        case -101:
            RCLCPP_INFO(this->get_logger(),"PCL Processor point limit reached");
            break;

        default:
            RCLCPP_INFO(this->get_logger(),"PCL Processor returned ERROR flag: %d", processReturnFlag);
            break;
        }
        filteredGround = pclProcessor.getCart();
        publishGroundFilteredPCL(0.0, 0.0, 0.0);
        pclProcessorMutex.unlock();
    }
    else{
        RCLCPP_INFO(this->get_logger(),"Pipeline mutex is locked, ignoring current pointcloud");
    }

    auto a2 = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Total pclCallback time in us: %lu", std::chrono::duration_cast<std::chrono::microseconds>(a2 - a1).count());
    
}

void OusterOffline::publihsDetectedCones(double xPos, double yPos, double yaw){
    Eigen::Matrix<float, Eigen::Dynamic, 1> xCones(conePos.rows()), yCones(conePos.rows());
    xCones = (conePos.col(0).array() * std::cos(yaw) - conePos.col(1).array() * std::sin(yaw) + xPos).matrix();
    yCones = (conePos.col(0).array() * std::sin(yaw) + conePos.col(1).array() * std::cos(yaw) + yPos).matrix();

    conesDetectedMsg.header.stamp = rclcpp::Node::now();
    conesDetectedMsg.width = conePos.rows();
    conesDetectedMsg.height = 1;
    conesDetectedMsg.row_step = conesDetectedMsg.width * conesDetectedMsg.point_step;
    conesDetectedMsg.data.resize(conesDetectedMsg.row_step);
    uint8_t* ptr = conesDetectedMsg.data.data();
    for(int i=0; i < xCones.size(); i++){
        *((float*)(ptr + i*conesDetectedMsg.point_step)) = xCones(i);
        *((float*)(ptr + i*conesDetectedMsg.point_step + 4)) = yCones(i);
        *((float*)(ptr + i*conesDetectedMsg.point_step + 8)) = 0.0;
    }
    pclConePosPublisher->publish(conesDetectedMsg);
}

void OusterOffline::publishGroundFilteredPCL(double xPos, double yPos, double yaw){
    filteredGroundMsg.header.stamp = rclcpp::Node::now();
    filteredGroundMsg.width = filteredGround.rows();
    filteredGroundMsg.height = 1;
    filteredGroundMsg.row_step = filteredGroundMsg.width * filteredGroundMsg.point_step;
    filteredGroundMsg.data.resize(filteredGroundMsg.row_step);
    RCLCPP_INFO(this->get_logger(),"Filtered ground PCL size = %u", filteredGround.rows());
    uint8_t* ptr = filteredGroundMsg.data.data();
    for(int i=0; i < filteredGround.rows(); i++){
        *((float*)(ptr + i*filteredGroundMsg.point_step)) = filteredGround(i,0);
        *((float*)(ptr + i*filteredGroundMsg.point_step + 4)) = filteredGround(i,1);
        *((float*)(ptr + i*filteredGroundMsg.point_step + 8)) = filteredGround(i,2);
    }
    pclGroundFilteredPublisher->publish(filteredGroundMsg);

}
