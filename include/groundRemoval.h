#ifndef GROUNDREMOVAL_H_INCLUDED
#define GROUNDREMOVAL_H_INCLUDED

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "utils.h"

struct Lines {
	Eigen::MatrixXf linesMatrix;
	Eigen::MatrixXi startPointsMatrix;
	Eigen::MatrixXi endPointsMatrix;
	Eigen::VectorXi sizes;
	
	Lines(int segments, int bins) { 
		linesMatrix = Eigen::MatrixXf::Zero(2*(bins - 1), segments);
		startPointsMatrix = Eigen::MatrixXi::Constant(bins - 1, segments, -1);
		endPointsMatrix = Eigen::MatrixXi::Constant(bins - 1, segments, -1);
		sizes = Eigen::VectorXi::Zero(segments);
	}
};

struct SegmentLines {
	Eigen::VectorXf linesMatrix;
	Eigen::VectorXi startPointsMatrix;
	Eigen::VectorXi endPointsMatrix;
	int size;
	
	SegmentLines(int bins) {
		linesMatrix = Eigen::VectorXf::Zero(2*(bins - 1));
		startPointsMatrix = Eigen::VectorXi::Constant(bins - 1, -1);
		endPointsMatrix = Eigen::VectorXi::Constant(bins - 1, -1);
		size = 0;
	}
	SegmentLines(const Lines &lines, int segment) {
		linesMatrix = lines.linesMatrix.col(segment);
		startPointsMatrix = lines.startPointsMatrix.col(segment);
		endPointsMatrix = lines.endPointsMatrix.col(segment);
		size = lines.sizes(segment);
	}
};

Eigen::Matrix <int, Eigen::Dynamic, 2, Eigen::RowMajor> calculatePartitionMatrix(const Eigen::MatrixXf &polar, int segments, int bins);
Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> calculatePrototypePoints(const Eigen::VectorXf &z, int segments, int bins, const Eigen::Matrix <int, Eigen::Dynamic, 2, Eigen::RowMajor> &partitionMatrix);
void filterCylindrical(Eigen::MatrixXf &polar, const float bounds[]);
Eigen::VectorXf getLine(const Eigen::VectorXf &properties);
Eigen::VectorXf updateLine(float x, float y, const Eigen::VectorXf &properties);
float distPointFromLine(float x, float y, const Eigen::VectorXf &param);
SegmentLines segmentGroundLinesFit(const Eigen::VectorXf &r, const Eigen::VectorXf &z, const Eigen::VectorXi &prototypePointsVector, const float* param);
Lines groundLinesFit(const Eigen::VectorXf &r, const Eigen::VectorXf &z, const Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &prototypePointsMatrix, const float param[]);
Eigen::Array <bool, Eigen::Dynamic, 1> groundClassifier(const Lines &lines, const Eigen::VectorXf &r, const Eigen::VectorXf &z, const Eigen::Matrix <int, Eigen::Dynamic, 2, Eigen::RowMajor> &partitionMatrix);
void filterGround(Eigen::MatrixXf &polar, const Eigen::Array <bool, Eigen::Dynamic, 1> &groundVector);

#endif //GROUNDREMOVAL_H_INCLUDED
