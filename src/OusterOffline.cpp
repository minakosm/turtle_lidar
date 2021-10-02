#include <memory>
#include <eigen3/Eigen/Dense>
#include <mutex>

#include "OusterOffline.h"
#include "rclcpp/qos.hpp"
// #include "turtle_common/fsmath_conversions.hpp"
#include "utils.h"

OusterOffline::OusterOffline(std::string filepath, 
                             std::string coneTrainXFilePath, 
                             std::string coneTrainYFilePath) : Node("ouster_offline"), 
                                                               pclProcessor(2048*10, filepath),
                                                               coneClassifier(),
                                                               cartesianAfterFilterGround(),
                                                               conePos() {
    // Print used configuration and train files
    RCLCPP_INFO(this->get_logger(), "Simulation LiDAR Node is using the following paths:\nConfiguration INI file: %s\nCone classifier TrainX: %s\nCone classifier TrainY: %s\n\n", filepath.c_str(), coneTrainXFilePath.c_str(), coneTrainYFilePath.c_str());

    // Initialize unique_ptr(s)
    X = std::make_unique<Eigen::VectorXf>();
    Y = std::make_unique<Eigen::VectorXf>();
    Z = std::make_unique<Eigen::VectorXf>();
    intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();

    // Cone classifier training
    Eigen::MatrixXf coneTrainDataX;
    Eigen::Matrix<int, Eigen::Dynamic, 1> coneTrainDataY;
    read_matrix<float>(coneTrainXFilePath, coneTrainDataX);
    read_vector<int>(coneTrainYFilePath, coneTrainDataY);
    Eigen::MatrixXf circleRadius = coneTrainDataX.col(0);
    Eigen::MatrixXf averageHeight = coneTrainDataX.col(1);
    Eigen::MatrixXf pRR = coneTrainDataX.col(2);
    int count = 0;
    if(pclProcessor.getClassifierSettings().useCircleRegression) {
        coneTrainDataX.col(count) = circleRadius;
        count++;
    }
    if(pclProcessor.getClassifierSettings().useAverageHeight) {
        coneTrainDataX.col(count) = averageHeight;
        count++;
    }
    if(pclProcessor.getClassifierSettings().usepRR) {
        coneTrainDataX.col(count) = pRR;
        count++;
    }
    coneTrainDataX.conservativeResize(Eigen::NoChange, count);
    coneClassifier.train(coneTrainDataX, coneTrainDataY);

    subsPubsInit();

    RCLCPP_INFO(this->get_logger(), "Simulation LiDAR processing driver initiated");
}

void OusterOffline::subsPubsInit() {
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

    pclAfterGroundFilterMsg.header.frame_id = "os1";
    pclAfterGroundFilterMsg.is_bigendian = false;
    pclAfterGroundFilterMsg.is_dense = true;
    pclAfterGroundFilterMsg.point_step = 12;
    pclAfterGroundFilterMsg.fields.resize(3);

    pclAfterGroundFilterMsg.fields[0].name = "x";
    pclAfterGroundFilterMsg.fields[0].offset = 0;
    pclAfterGroundFilterMsg.fields[0].datatype = 7;
    pclAfterGroundFilterMsg.fields[0].count = 1;

    pclAfterGroundFilterMsg.fields[1].name = "y";
    pclAfterGroundFilterMsg.fields[1].offset = 4;
    pclAfterGroundFilterMsg.fields[1].datatype = 7;
    pclAfterGroundFilterMsg.fields[1].count = 1;

    pclAfterGroundFilterMsg.fields[2].name = "z";
    pclAfterGroundFilterMsg.fields[2].offset = 8;
    pclAfterGroundFilterMsg.fields[2].datatype = 7;
    pclAfterGroundFilterMsg.fields[2].count = 1;

    // conesDetectedMsg.fields[3].name = "intensity";
    // conesDetectedMsg.fields[3].offset = 12;
    // conesDetectedMsg.fields[3].datatype = 4;
    // conesDetectedMsg.fields[3].count = 1;

    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    pclConePublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/conesDetectedOffline", 10);
    pclGroundFilteredPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/groundFiltered", 10);
    pclSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/rawPointcloud", qos, std::bind(&OusterOffline::pclCallback, this, std::placeholders::_1));
}

void OusterOffline::pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "PointCloud2 Message received START");
    auto a1 = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "\nPointCloud2 size = %u", msg->width*msg->height);
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
            publishDetectedCones(0.0, 0.0, 0.0);
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
        cartesianAfterFilterGround = pclProcessor.getCart();
        publishGroundFilteredPCL(0.0, 0.0, 0.0);
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

void OusterOffline::publishGroundFilteredPCL(double xPos, double yPos, double yaw) {
    pclAfterGroundFilterMsg.header.stamp = rclcpp::Node::now();
    pclAfterGroundFilterMsg.width = cartesianAfterFilterGround.rows();
    pclAfterGroundFilterMsg.height = 1;
    pclAfterGroundFilterMsg.row_step = pclAfterGroundFilterMsg.width * pclAfterGroundFilterMsg.point_step;
    pclAfterGroundFilterMsg.data.resize(pclAfterGroundFilterMsg.row_step);
    RCLCPP_INFO(this->get_logger(), "Filtered ground PCL size = %u", cartesianAfterFilterGround.rows());
    uint8_t* ptr = pclAfterGroundFilterMsg.data.data();
    for(int i = 0; i < cartesianAfterFilterGround.rows(); i++) {
        *((float*)(ptr + i*pclAfterGroundFilterMsg.point_step)) = cartesianAfterFilterGround(i,0);
        *((float*)(ptr + i*pclAfterGroundFilterMsg.point_step + 4)) = cartesianAfterFilterGround(i,1);
        *((float*)(ptr + i*pclAfterGroundFilterMsg.point_step + 8)) = cartesianAfterFilterGround(i,2);
        // *((uint16_t*)(ptr + i*pclAfterGroundFilterMsg.point_step + 12)) = 20;
    }
    pclGroundFilteredPublisher->publish(pclAfterGroundFilterMsg);
}

void OusterOffline::publishDetectedCones(double xPos, double yPos, double yaw) {
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
        // RCLCPP_INFO(this->get_logger(), "xCone = %f   yCone = %f", xCones(i), yCones(i));
    }
    pclConePublisher->publish(conesDetectedMsg);
}
