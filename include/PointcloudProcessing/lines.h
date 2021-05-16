#ifndef LINES_H
#define LINES_H

#include <ostream>
#include <eigen3/Eigen/Dense>

struct Lines {
	Eigen::MatrixXf linesMatrix;
	Eigen::MatrixXf linesMedianMatrixX;     
	Eigen::MatrixXf linesMedianMatrixY;
	Eigen::VectorXi sizes;
	
	Lines(int segments, int bins) {
		linesMatrix = Eigen::MatrixXf::Zero(2*(bins - 1), segments);
		linesMedianMatrixX = Eigen::MatrixXf::Zero(bins - 1, segments);
		linesMedianMatrixY = Eigen::MatrixXf::Zero(bins - 1, segments);
		sizes = Eigen::VectorXi::Zero(segments);
	}
};

// Struct that holds information about a segment's regressed lines
struct SegmentLines {
	Eigen::VectorXf linesMatrix;            // Vector that holds the values of m,b from each line regressed in the segment (y = m*x + b) 
	Eigen::VectorXf linesMedianMatrixX;     // Vector that contains the X coordinates of median points that come up from the lines' start points and end points
	Eigen::VectorXf linesMedianMatrixY;     // Vector that contains the Y coordinates of median points that come up from the lines' start points and end points
	int size;                               // Number of lines regressed in the segment
	
	SegmentLines(int bins) {
		linesMatrix = Eigen::VectorXf::Zero(2*(bins - 1));
		linesMedianMatrixX = Eigen::VectorXf::Zero(bins - 1);
		linesMedianMatrixY = Eigen::VectorXf::Zero(bins - 1);
		size = 0;
	}
	SegmentLines(const Lines &lines, int segment) {
		linesMatrix = lines.linesMatrix.col(segment);
		linesMedianMatrixX = lines.linesMedianMatrixX.col(segment);
		linesMedianMatrixY = lines.linesMedianMatrixY.col(segment);
		size = lines.sizes(segment);
	}
};

std::ostream &operator<<(std::ostream &os, SegmentLines const &line) { 
    return os << "SegmentLines Lines Matrix =\n" << line.linesMatrix << std::endl << std::endl << "SegmentLines Lines MedianX =\n" 
              << line.linesMedianMatrixX << std::endl << std::endl << "SegmentLines Lines MedianY =\n" << line.linesMedianMatrixY 
              << std::endl << std::endl <<"SegmentLines size = " << line.size << std::endl << std::endl;
}

std::ostream &operator<<(std::ostream &os, Lines const &lines) { 
    return os << "Lines Matrix =\n" << lines.linesMatrix << std::endl << std::endl << "Lines MedianX =\n" 
              << lines.linesMedianMatrixX << std::endl << std::endl << "Lines MedianY =\n" << lines.linesMedianMatrixY 
              << std::endl << std::endl <<"Lines size = " << lines.sizes << std::endl << std::endl;
}

#endif // LINES_H_INCLUDED
