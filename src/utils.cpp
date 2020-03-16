/* Library .cpp file containing utility functions:
 1) removeRows(Matrix <T, Dynamic, Dynamic> &matrix, Array <bool, Dynamic, 1> logicalVector) ->
    Pass an Eigen matrix and a same-row-count boolean vector containing true/false on indexes
    that you want to keep or remove from the matrix
 2) pol2cart(const VectorXf &azim, const VectorXf &r, const VectorXf &z) -> Convert cylindrical 
    coordinates to cartesian (currently only for coordinates stored in float datatype)
 3) cart2pol(const VectorXf &x, const VectorXf &y, const VectorXf &z) -> Convert cartesian 
    coordinates to cylindrical (currently only for coordinates stored in float datatype)
 4) pointDistance(T x1, T y1, T x2, T y2) -> Return distance between two points in 2D space
*/

#include "utils.h"

Eigen::Matrix <float, Eigen::Dynamic, 3> pol2cart(const Eigen::VectorXf &azim, const Eigen::VectorXf &r, const Eigen::VectorXf &z) {
	Eigen::MatrixXf cart(azim.size(), 3);
	cart.col(0) = (r.array() * (azim.array().cos())).matrix();
	cart.col(1) = (r.array() * (azim.array().sin())).matrix();
	cart.col(2) = z;
	return cart;
}

Eigen::Matrix <float, Eigen::Dynamic, 3> cart2pol(const Eigen::VectorXf &x, const Eigen::VectorXf &y, const Eigen::VectorXf &z) {
	Eigen::MatrixXf coord(x.size(), 3);
    #pragma omp parallel for num_threads(2)
		for (int i = 0; i < x.size(); i++) {
			coord(i,0) = std::atan2(y(i), x(i));
		}
	coord.col(1) = (x.array().square() + y.array().square()).sqrt().matrix();
	coord.col(2) = z;
	return coord;
}
