#include <cmath>
#include <vector>
#include "groundRemoval.h"

using namespace Eigen;

Matrix <int, Dynamic, 2, RowMajor> calculatePartitionMatrix(const MatrixXf &polar, int segments, int bins) {
	Matrix <int, Dynamic, Dynamic, RowMajor> partitionMatrix(polar.rows(), 2);
	float minAzim = polar.col(0).minCoeff();
	float maxAzim = polar.col(0).maxCoeff();
	float minRad = polar.col(1).minCoeff();
	float maxRad = polar.col(1).maxCoeff();
	float shiftAngle = (maxAzim - minAzim) / ((float)segments);
	float shiftRad = (maxRad - minRad) / ((float)bins);
	partitionMatrix.col(0) = ((polar.col(0).array() - minAzim) / shiftAngle).floor().cast<int>().matrix();
	partitionMatrix.col(1) = ((polar.col(1).array() - minRad) / shiftRad).floor().cast<int>().matrix();
	partitionMatrix.col(0) = (partitionMatrix.col(0).array() > (segments - 1)).select(segments - 1, partitionMatrix.col(0));
	partitionMatrix.col(1) = (partitionMatrix.col(1).array() > (bins - 1)).select(bins - 1, partitionMatrix.col(1));
	return partitionMatrix;
}

Matrix <int, Dynamic, Dynamic, RowMajor> calculatePrototypePoints(const VectorXf &z, int segments, int bins, const Matrix <int, Dynamic, 2, RowMajor> &partitionMatrix) {
	Matrix <int, Dynamic, Dynamic, RowMajor> prototypePoints = MatrixXi::Constant(segments, bins, -1);
	MatrixXf prototypePointsDistances = MatrixXf::Constant(segments, bins, 100000.0f);
	for(unsigned int k = 0; k < z.size(); k++) {
		int seg = partitionMatrix(k,0);
		int bin = partitionMatrix(k,1);
		if (z(k) < prototypePointsDistances(seg, bin)) {
			prototypePointsDistances(seg, bin) = z(k);
			prototypePoints(seg, bin) = k;
		}
	}
	return prototypePoints;
}

void filterCylindrical(MatrixXf &polar, const float bounds[]) {
	removeRows <float>(polar, ((polar.col(0).array() > bounds[0]).cast<int>() + (polar.col(0).array() < bounds[1]).cast<int>() + 
					           (polar.col(1).array() > bounds[2]).cast<int>() + (polar.col(1).array() < bounds[3]).cast<int>() + 
					           (polar.col(2).array() > bounds[4]).cast<int>() + (polar.col(2).array() < bounds[5]).cast<int>()).cast<bool>());
}

VectorXf getLine(const VectorXf &properties) {
	VectorXf line(3);
	line(0) = properties(4) / properties(2);
	line(1) = properties(1) - line(0) * properties(0);
	line(2) = sqrtf(1 - (properties(4) / sqrtf(properties(2) * properties(3)))*(properties(4) / sqrtf(properties(2) * properties(3)))) * sqrtf(properties(3));
	return line;
}

VectorXf updateLine(float x, float y, const VectorXf &properties) {
	VectorXf newProperties(6);
	float dx = x - properties(0);
	float dy = y - properties(1);
	newProperties(2) = properties(2) + (properties(5)/(properties(5)+1) * dx * dx - properties(2)) / (properties(5)+1);
	newProperties(3) = properties(3) + (properties(5)/(properties(5)+1) * dy * dy - properties(3)) / (properties(5)+1);
	newProperties(4) = properties(4) + (properties(5)/(properties(5)+1) * dx * dy - properties(4)) / (properties(5)+1);
    newProperties(0) = properties(0) + dx/(properties(5)+1);
    newProperties(1) = properties(1) + dy/(properties(5)+1);
    newProperties(5) = properties(5) + 1;
    return newProperties;
}

float distPointFromLine(float x, float y, const VectorXf &param) {
	return std::abs(-param(0)*x+y-param(1)) / sqrt(param(0)*param(0)+1);
}

SegmentLines segmentGroundLinesFit(const VectorXf &r, const VectorXf &z, const VectorXi &prototypePointsVector, const float* param) {
	float Tm = param[0];      		// tan(20deg) = 0.363970234
	float Tmsmall = param[1];  		// tan(2deg) = 0.034920769
    float Tbmax = param[2];         // +0.3 from -0.3
    float Tbmin = param[3];         // -0.25 from -0.3
    float Trmse = param[4];
    float Tdprev = param[5];
    
    SegmentLines lineVector(prototypePointsVector.size());
    std::vector <int> Pl;
    Pl.reserve(prototypePointsVector.cols());
    VectorXf currentLineProperties = Matrix <float, 6, 1>::Zero();
    VectorXf previousLineParam = Matrix <float, 3, 1>::Constant(10000.0f);
    for(int i = 0; i < prototypePointsVector.size(); i++) {
		int point = prototypePointsVector(i);
		if (point != -1) {
			if(Pl.size() > 1) {
				VectorXf potentialLine = getLine(updateLine(r(point), z(point), currentLineProperties));
				if ((std::abs(potentialLine(0)) < Tm) && ((potentialLine(0) > Tmsmall) ||  ((potentialLine(1) < Tbmax) && (potentialLine(1) > Tbmin))) && (potentialLine(2) < Trmse)) {
					Pl.push_back(point);
					currentLineProperties = updateLine(r(point), z(point), currentLineProperties);
				}
				else {
					potentialLine = getLine(currentLineProperties);
					previousLineParam = potentialLine;
					lineVector.linesMatrix(2*lineVector.size) = potentialLine(0);
					lineVector.linesMatrix(2*lineVector.size + 1) = potentialLine(1);
					i--;
					lineVector.endPointsMatrix(lineVector.size) = prototypePointsVector(i);
					lineVector.size++;
					Pl.erase(Pl.begin(), Pl.end());
				}
			}
			else { 
				if(((distPointFromLine(r(point), z(point), previousLineParam)) < Tdprev) || (lineVector.size == 0) || (Pl.size() != 0)) {
					if (Pl.size() == 0)
						lineVector.startPointsMatrix(lineVector.size) = point;
                    Pl.push_back(point);
                    currentLineProperties = updateLine(r(point), z(point), currentLineProperties);
				}
			}
		}
	}
    if (Pl.size() > 1) {
        previousLineParam = getLine(currentLineProperties);
        lineVector.linesMatrix(2*lineVector.size) = previousLineParam(0);
        lineVector.linesMatrix(2*lineVector.size + 1) = previousLineParam(1);
        lineVector.endPointsMatrix(lineVector.size) = Pl.at(Pl.size() - 1);
        lineVector.size++;
	}
    return lineVector;
}

Lines groundLinesFit(const VectorXf &r, const VectorXf &z, const Matrix <int, Dynamic, Dynamic, RowMajor> &prototypePointsMatrix, const float param[]) {
	Lines lines(prototypePointsMatrix.rows(), prototypePointsMatrix.cols());
	//~ #pragma omp parallel for num_threads(2)
		for (int i = 0; i < prototypePointsMatrix.rows(); i++) {
			SegmentLines line = segmentGroundLinesFit(r, z, prototypePointsMatrix.row(i), param);
			//~ std::cout << line.linesMatrix << endl << endl << line.startPointsMatrix << endl << endl << line.endPointsMatrix << endl << endl << line.size << endl;
			lines.linesMatrix.col(i) = line.linesMatrix;
			lines.startPointsMatrix.col(i) = line.startPointsMatrix;
			lines.endPointsMatrix.col(i) = line.endPointsMatrix;
			lines.sizes(i) = line.size;
		}
	//~ std::cout << lines.linesMatrix << endl << endl << lines.startPointsMatrix << endl << endl << lines.endPointsMatrix << endl << endl << lines.sizes << endl << endl;
	return lines;
}

Array <bool, Dynamic, 1> groundClassifier(const Lines &lines, const VectorXf &r, const VectorXf &z, const Matrix <int, Dynamic, 2, RowMajor> &partitionMatrix) {
	float Tdground = 0.03;
	Array <bool, Dynamic, 1> groundArray = Array<bool, Dynamic, 1>::Constant(partitionMatrix.rows(),1, false);
	#pragma omp parallel for num_threads(2)
		for(int i = 0; i < r.rows(); i++) {
			int segment = partitionMatrix(i,0);
			SegmentLines lineVector(lines, segment);
			//~ std::cout << lineVector.linesMatrix << endl << lineVector.startPointsMatrix << endl << lineVector.endPointsMatrix << endl << lineVector.size << endl;
			int lineSelection = 0;
			if(lineVector.size > 1) {
				float temp1 = pointDistance <float>(r(lineVector.startPointsMatrix(0)), z(lineVector.startPointsMatrix(0)), r(i), z(i));
				float temp2 = pointDistance <float>(r(lineVector.endPointsMatrix(0)), z(lineVector.endPointsMatrix(0)), r(i), z(i));
				float dist[2] = {std::min(temp1, temp2), std::max(temp1, temp2)};
				for(int j = 1; j < lineVector.size; j++) {
					float a = pointDistance <float>(r(lineVector.startPointsMatrix(j)), z(lineVector.startPointsMatrix(j)), r(i), z(i));
					float b = pointDistance <float>(r(lineVector.endPointsMatrix(j)), z(lineVector.endPointsMatrix(j)), r(i), z(i));
					temp1 = std::min(a, b);
					temp2 = std::max(a, b);
					if(temp1 < dist[0]) {
						dist[0] = temp1;
						dist[1] = temp2;
						lineSelection = j;
					}
					else if (temp1 == dist[0]) {
						if(temp2 < dist[1]) {
							dist[1] = temp2;
							lineSelection = j;
						}
					}
				}
			}
			if(distPointFromLine(r(i), z(i), lineVector.linesMatrix.segment(2*lineSelection, 2*(lineSelection + 1))) < Tdground)
				groundArray(i) = true;
		}
	return groundArray;
}

void filterGround(MatrixXf &polar, const Array <bool, Dynamic, 1> &groundVector) {
	removeRows <float>(polar, groundVector);
}
