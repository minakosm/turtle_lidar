#include "utils.h"
#include "groundRemoval.h"
#include "readMatrix.h"
#include "readINI.h"
#include <iostream>
#include <chrono>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

int main() {    
    MatrixXf data;
//	string filename;
//	cin >> filename;
	read_matrix <float> ("../example/lidar.txt", data);
	MatrixXf polar(data.col(0).size(),3);
	VectorXf x(data.col(0).size());
	VectorXf y(data.col(0).size());
	VectorXf z(data.col(0).size());
	x = data.col(0);
	y = data.col(1);
	z = data.col(2);
	int segments;
	int bins;
	float filterBounds[6];
	float groundParams[6];
	readINIfile(filterBounds, groundParams, segments, bins);
	
	auto a1 = chrono::high_resolution_clock::now();
	polar = cart2pol(x, y, z);
	auto a2 = chrono::high_resolution_clock::now();
	cout << "--cart2pol duration = " << chrono::duration_cast<chrono::microseconds>( a2 - a1 ).count() << "μs" << endl;
	cout << "Before filtering polar = " << polar.rows() << "x" << polar.cols() << endl;
	
	auto b1 = chrono::high_resolution_clock::now();
	filterCylindrical(polar, filterBounds);
	auto b2 = chrono::high_resolution_clock::now();
	cout << "--filterCylindrical duration = " << chrono::duration_cast<chrono::microseconds>( b2 - b1 ).count() << "μs" << endl;
	cout << "After filtering polar = " << polar.rows() << "x" << polar.cols() << endl;
	
	auto c1 = chrono::high_resolution_clock::now();
	Matrix <int, Dynamic, 2, RowMajor> partitionMatrix = calculatePartitionMatrix(polar, segments, bins);
	auto c2 = chrono::high_resolution_clock::now();
	cout << "--partitionMatrix calculation duration = " << chrono::duration_cast<chrono::microseconds>( c2 - c1 ).count() << "μs" << endl;
	
	auto d1 = chrono::high_resolution_clock::now();
	Matrix <int, Dynamic, Dynamic, RowMajor> prototypePointsMatrix = calculatePrototypePoints(polar.col(2), segments, bins, partitionMatrix);
	auto d2 = chrono::high_resolution_clock::now();
	cout << "--prototypePointsMatrix calculation duration = " << chrono::duration_cast<chrono::microseconds>( d2 - d1 ).count() << "μs" << endl;
	
	auto e1 = chrono::high_resolution_clock::now();
	Lines linesMatrix = groundLinesFit(polar.col(1), polar.col(2), prototypePointsMatrix, groundParams);
	auto e2 = chrono::high_resolution_clock::now();
	cout << "--groundLinesFit duration = " << chrono::duration_cast<chrono::microseconds>( e2 - e1 ).count() << "μs" << endl;
	
	auto f1 = chrono::high_resolution_clock::now();
	Array <bool, Dynamic, 1>  groundVector = groundClassifier(linesMatrix, polar.col(1), polar.col(2), partitionMatrix);
	auto f2 = chrono::high_resolution_clock::now();
	cout << "--groundClassifier duration = " << chrono::duration_cast<chrono::microseconds>( f2 - f1 ).count() << "μs" << endl;
	
	auto g1 = chrono::high_resolution_clock::now();
	filterGround(polar, groundVector);
	auto g2 = chrono::high_resolution_clock::now();
	cout << "--filterGround duration = " << chrono::duration_cast<chrono::microseconds>( g2 - g1 ).count() << "μs" << endl;
	cout << "After filtering ground polar = " << polar.rows() << "x" << polar.cols() << endl;
	
	cout << "Total duration = " << chrono::duration_cast<chrono::microseconds>( g2 - g1 ).count() +
								   chrono::duration_cast<chrono::microseconds>( f2 - f1 ).count() + 
								   chrono::duration_cast<chrono::microseconds>( e2 - e1 ).count() +
								   chrono::duration_cast<chrono::microseconds>( d2 - d1 ).count() +
								   chrono::duration_cast<chrono::microseconds>( c2 - c1 ).count() + 
								   chrono::duration_cast<chrono::microseconds>( b2 - b1 ).count() + 
								   chrono::duration_cast<chrono::microseconds>( a2 - a1 ).count() << "μs" << endl;
    return 0;
}
