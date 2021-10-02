#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "PointcloudProcessing.h"
#include "utils.h"
#include "settings.h"
#include "bayes.h"

int main() {
    omp_set_num_threads(NUM_OF_THREADS);

    PointcloudProcessing pcl("../../lidarConfig.ini");
    Eigen::Matrix <float, Eigen::Dynamic, Eigen::Dynamic> mat;
    read_matrix("../../example/lidar.txt", mat);

    std::unique_ptr<Eigen::VectorXf> X = std::make_unique<Eigen::VectorXf>();
    std::unique_ptr<Eigen::VectorXf> Y = std::make_unique<Eigen::VectorXf>();
    std::unique_ptr<Eigen::VectorXf> Z = std::make_unique<Eigen::VectorXf>();
    std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>();
    Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos;
    (*X) = mat.col(0);
    (*Y) = mat.col(1);
    (*Z) = mat.col(2);
    intensities->resize(X->size());
    
    Eigen::MatrixXf coneTrainDataX;
    Eigen::Matrix<int, Eigen::Dynamic, 1> coneTrainDataY;
    read_matrix<float>("../../example/simConeTrainDataX.txt", coneTrainDataX);
    read_vector<int>("../../example/simConeTrainDataY.txt", coneTrainDataY);
    GNBC nbc(coneTrainDataX, coneTrainDataY);

    std::cout << "Reached pipeline section with a PCL of size: " << X->size() << "\n";

    pcl.pipeline(X, Y, Z, intensities, nbc, conePos, 20000, 200);
}