#include "settings.h"
#include "PointcloudProcessing.h"
#include "lines.h"
#include "utils.h"
#include "bayes.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>
#include <memory>

#include "fastcluster.h"
#include "dbscan.h"

using namespace std;
using namespace Eigen;

int main() {
    Matrix <float, Dynamic, Dynamic> data;
	read_matrix<float>("../example/lidar.txt",data);
	
	MatrixXf trainDataX;
	VectorXi trainDataY;
	read_matrix<float>("../example/coneTrainDataX.txt", trainDataX);
	read_vector<int>("../example/coneTrainDataY.txt", trainDataY);
	
	std::unique_ptr<Eigen::VectorXf> x = std::make_unique<Eigen::VectorXf>(data.rows());
	std::unique_ptr<Eigen::VectorXf> y = std::make_unique<Eigen::VectorXf>(data.rows());
	std::unique_ptr<Eigen::VectorXf> z = std::make_unique<Eigen::VectorXf>(data.rows());
    std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensities = std::make_unique<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>>(data.rows());

	*x = data.col(0);
	*y = data.col(1);
	*z = data.col(2);
	
	auto a1 = chrono::high_resolution_clock::now();
	GNBC nbc(trainDataX, trainDataY);
	auto a2 = chrono::high_resolution_clock::now();
	cout << "--GNBC declaration and training duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	a1 = chrono::high_resolution_clock::now();
	PointcloudProcessing lidarPipeline;
	Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> conePos = lidarPipeline.pipeline(x, y, z, intensities, nbc);
	a2 = chrono::high_resolution_clock::now();
	cout << "--Pipeline duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;

    cout << "Number of cones detected: " << conePos.rows() << endl;

	return 0;
}
