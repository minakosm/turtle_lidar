#include "settings.h"
#include "PointcloudProcessing.h"
#include "lines.h"
#include "utils.h"
#include "bayes.h"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>

#include "fastcluster.h"
#include "dbscan.h"

using namespace std;
using namespace Eigen;

int main() {
	Matrix <float, Dynamic, Dynamic> data;
	read_matrix<float>("../example/lidar.txt",data);
	
	VectorXf *x = new VectorXf(data.rows());
	VectorXf *y = new VectorXf(data.rows());
	VectorXf *z = new VectorXf(data.rows());
    Matrix<uint16_t, Dynamic, 1> *inten = new Matrix<uint16_t, Dynamic, 1>(data.rows());
	*x = data.col(0);
	*y = data.col(1);
	*z = data.col(2);
    *inten = Matrix<uint16_t, Dynamic, 1>::Zero(data.rows());
	
	auto a1 = chrono::high_resolution_clock::now();
	PointcloudProcessing pcl(x,y,z,inten);
	auto a2 = chrono::high_resolution_clock::now();
	cout << "--Declaration duration = " << chrono::duration_cast<chrono::microseconds>(a2 - a1).count() << "μs" << endl;
	
	//pcl.printSettings();
	pcl.printPointcloudSize();
	
	auto b1 = chrono::high_resolution_clock::now();
	pcl.filter();
	auto b2 = chrono::high_resolution_clock::now();
	cout << "--Filter duration = " << chrono::duration_cast<chrono::microseconds>(b2 - b1).count() << "μs" << endl;
	pcl.printPointcloudSize();
	//cout << *pcl.x << endl << endl << *pcl.y << endl << endl << *pcl.z << endl << endl << *pcl.azim << endl << endl << *pcl.r;
	
	auto c1 = chrono::high_resolution_clock::now();
	pcl.calculatePartitionMatrix();
	auto c2 = chrono::high_resolution_clock::now();
	cout << "--Partition Matrix duration = " << chrono::duration_cast<chrono::microseconds>(c2 - c1).count() << "μs" << endl;
	
	auto d1 = chrono::high_resolution_clock::now();
	pcl.calculatePrototypePointsMatrix();
	auto d2 = chrono::high_resolution_clock::now();
	cout << "--Prototype Matrix duration = " << chrono::duration_cast<chrono::microseconds>(d2 - d1).count() << "μs" << endl;
	
	//cout << pcl.prototypePointsMatrix << std::endl;
	
	auto e1 = chrono::high_resolution_clock::now();
	pcl.groundLinesFit();
	auto e2 = chrono::high_resolution_clock::now();
	cout << "--Ground Lines Fit duration = " << chrono::duration_cast<chrono::microseconds>(e2 - e1).count() << "μs" << endl;
	
	//cout << pcl.lines << endl;
	
	auto f1 = chrono::high_resolution_clock::now();
	pcl.groundClassifier();
	auto f2 = chrono::high_resolution_clock::now();
	cout << "--Ground Classifying duration = " << chrono::duration_cast<chrono::microseconds>(f2 - f1).count() << "μs" << endl;
	
	auto g1 = chrono::high_resolution_clock::now();
	pcl.filterGround();
	auto g2 = chrono::high_resolution_clock::now();
	cout << "--Filter Ground duration = " << chrono::duration_cast<chrono::microseconds>(g2 - g1).count() << "μs" << endl;
	
	cout << "Non ground size = " << (pcl.getCart()).rows() << endl;
	
	auto h1 = chrono::high_resolution_clock::now();
	pcl.nonGroundClustering();
	auto h2 = chrono::high_resolution_clock::now();
	cout << "--Clustering duration = " << chrono::duration_cast<chrono::microseconds>(h2 - h1).count() << "μs" << endl;

	MatrixXf trainDataX;
	VectorXi trainDataY;
	read_matrix<float>("../example/coneTrainDataX.txt", trainDataX);
	read_vector<int>("../example/coneTrainDataY.txt", trainDataY);
	
	auto i1 = chrono::high_resolution_clock::now();
	GNBC nbc(trainDataX, trainDataY);
	auto i2 = chrono::high_resolution_clock::now();
	cout << "--GNBC declaration and training duration = " << chrono::duration_cast<chrono::microseconds>(i2 - i1).count() << "μs" << endl;
    
    auto j1 = chrono::high_resolution_clock::now();
    Matrix <float, Dynamic, 2, RowMajor> conePos;
    pcl.clusterClassifier(nbc, conePos);
    auto j2 = chrono::high_resolution_clock::now();
    cout << "--Classifier duration = " << chrono::duration_cast<chrono::microseconds>(j2 - j1).count() << "μs" << endl;

    cout << "TOTAL TIME: " << (float)(chrono::duration_cast<chrono::microseconds>(a2 - a1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(b2 - b1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(c2 - c1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(d2 - d1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(e2 - e1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(f2 - f1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(g2 - g1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(h2 - h1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(i2 - i1).count() + 
                                      chrono::duration_cast<chrono::microseconds>(j2 - j1).count()) / 1000 << "ms" << endl;
	//std::ofstream file1("groundArray.txt");
	//if (file1.is_open())
		//file1 << pcl.groundArray;
    //file1.close();
    
    std::ofstream file2("cart.txt");
	if (file2.is_open())
		file2 << (pcl.getCart());
    file2.close();
		
	std::ofstream file3("clusters.txt");
	if (file3.is_open())
		file3 << (pcl.getClusters());
    file3.close();
		
	std::ofstream file4("conePos.txt");
	if (file4.is_open())
		file4 << conePos;
    file4.close();

	return 0;
}
