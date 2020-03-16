#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <iostream>
#include <eigen3/Eigen/Dense>

template <class T> T pointDistance(T x1, T y1, T x2, T y2) {
	return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

template <class T> void removeRows(Eigen::Matrix <T, Eigen::Dynamic, Eigen::Dynamic> &matrix,const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector) {
	unsigned int size = (logicalVector == 0).count();
	unsigned int counter = 0;
	for(unsigned int i = 0; i < matrix.rows(); i++) {
		if(logicalVector(i) == 0) {
			matrix.row(counter) = matrix.row(i);
			counter++;
		}
	}
	matrix.conservativeResize(size, matrix.cols());
}

Eigen::Matrix <float, Eigen::Dynamic, 3> pol2cart(const Eigen::VectorXf &azim, const Eigen::VectorXf &r, const Eigen::VectorXf &z);
Eigen::Matrix <float, Eigen::Dynamic, 3> cart2pol(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const Eigen::VectorXf &z);

#endif //UTILS_H_INCLUDED
