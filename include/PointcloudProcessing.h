#ifndef POINTCLOUD_PROCESSING_H
#define POINTCLOUD_PROCESSING_H

#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <memory>
#include <fstream>

#include "settings.h"
#include "lines.h"
#include "bayes.h"
#include "utils.h"

class PointcloudProcessing {
    private:
        int segments;
        int bins;
        BaseFilter bF;
        GroundFilter gF;
        ClusterSettings cS;
        ClassifierSettings clS;

        std::unique_ptr<Eigen::VectorXf> x,y,z,azim,r;
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor>> cart;

        Eigen::Matrix <uint8_t, Eigen::Dynamic, 2, Eigen::RowMajor> partitionMatrix;
        Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prototypePointsMatrix;
        Lines lines;
        Eigen::Array <bool, Eigen::Dynamic, 1> groundArray;
        Eigen::VectorXd condensedClusterDistances;
        Eigen::VectorXi clusters;
        int numberOfClusters;
        Eigen::Array <bool, Eigen::Dynamic, 1> coneClusters;
        Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> clusterPositions;
    public:
        PointcloudProcessing();
        PointcloudProcessing(Eigen::VectorXf *X, Eigen::VectorXf *Y, Eigen::VectorXf *Z);
        void printPointcloudSize();
        void printSettings(); 
        
        Eigen::VectorXf getX();
        Eigen::VectorXf getY();
        Eigen::VectorXf getZ();
        Eigen::VectorXf getAzim();
        Eigen::VectorXf getR();
        Eigen::VectorXi getClusters();
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getCart();

        void filter();
        void removePoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector);
        void calculatePartitionMatrix();
        void calculatePrototypePointsMatrix();
        void checkPrototypePointsMatrix();
        void groundLinesFit();
        float distPointFromLine(float x, float y, const Eigen::VectorXf &param);
        Eigen::VectorXf getLine(const Eigen::VectorXf &properties);
        Eigen::VectorXf updateLine(float x, float y, const Eigen::VectorXf &properties);
        SegmentLines segmentGroundLinesFit(int segment);
        void groundClassifier();
        void filterGround();
        void nonGroundClustering();
        void condensedDistances();
        void hierarchicalClustering();
        void DBSCANclustering();
        void clustersVectorFun();
        Eigen::Array <bool, Eigen::Dynamic, 1> clusterClassifier(GNBC &nbc);
        Eigen::Vector3f regressCircle(const Eigen::VectorXf &xC, const Eigen::VectorXf &yC);
};

#endif // POINTCLOUD_PROCESSING_H_INCLUDED
