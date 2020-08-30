#include "settings.h"
#include "PointcloudProcessing.h"
#include "lines.h"
#include "utils.h"
#include "bayes.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace Eigen;

int main() {
	Matrix <float, Dynamic, Dynamic> data;
	read_matrix<float>("../example/lidar1.txt",data);
	
	VectorXf *x = new VectorXf(data.rows());
	VectorXf *y = new VectorXf(data.rows());
	VectorXf *z = new VectorXf(data.rows());
	*x = data.col(0);
	*y = data.col(1);
	*z = data.col(2);
	
	auto a1 = chrono::high_resolution_clock::now();
	PointcloudProcessing pcl(x,y,z);
	auto a2 = chrono::high_resolution_clock::now();
	cout << "--Declaration duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	//pcl.printSettings();
	pcl.printPointcloudSize();
	
	a1 = chrono::high_resolution_clock::now();
	pcl.filter();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Filter duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	pcl.printPointcloudSize();
	//cout << *pcl.x << endl << endl << *pcl.y << endl << endl << *pcl.z << endl << endl << *pcl.azim << endl << endl << *pcl.r;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.calculatePartitionMatrix();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Partition Matrix duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.calculatePrototypePointsMatrix();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Prototype Matrix duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	//cout << pcl.prototypePointsMatrix << std::endl;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.groundLinesFit();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Ground Lines Fit duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	//cout << pcl.lines << endl;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.groundClassifier();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Ground Classifying duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.filterGround();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Filter Ground duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	cout << "Non ground size = " << (pcl.getCart()).rows() << endl;
	
	a1 = chrono::high_resolution_clock::now();
	pcl.nonGroundClustering();
	a2 = chrono::high_resolution_clock::now();
	cout << "--Clustering duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;

	MatrixXf trainDataX;
	VectorXi trainDataY;
	read_matrix<float>("../gaussian-naive-bayes-classifier/trainDataX.txt", trainDataX);
	read_vector<int>("../gaussian-naive-bayes-classifier/trainDataY.txt", trainDataY);
	
	a1 = chrono::high_resolution_clock::now();
	GNBC nbc(trainDataX, trainDataY);
	a2 = chrono::high_resolution_clock::now();
	cout << "--GNBC declaration and training duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
    
    a1 = chrono::high_resolution_clock::now();
    Eigen::Array<bool, Eigen::Dynamic, 1> cones = pcl.clusterClassifier(nbc);
    a2 = chrono::high_resolution_clock::now();
    cout << "--Classifier duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;

	//std::ofstream file1("groundArray.txt");
	//if (file1.is_open())
		//file1 << pcl.groundArray;
    
    std::ofstream file2("cart.txt");
	if (file2.is_open())
		file2 << (pcl.getCart());
		
	std::ofstream file3("clusters.txt");
	if (file3.is_open())
		file3 << (pcl.getClusters());
	
	return 0;
}
