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
        float minAzim, maxAzim, minRad, maxRad, shiftAngle, shiftRad;
        BaseFilter baseFilter;
        GroundFilter groundFilter;
        ClusterSettings clusterSettings;
        ClassifierSettings classifierSettings;

        std::unique_ptr<Eigen::VectorXf> x,y,z,azim,r;
        std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensity;
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
        
        void polarInit();
        bool removePoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector);
        Eigen::VectorXf getLine(const Eigen::VectorXf &properties);
        Eigen::VectorXf updateLine(float x, float y, const Eigen::VectorXf &properties);
        SegmentLines segmentGroundLinesFit(int segment);
        inline float distPointFromLine(float x, float y, const Eigen::VectorXf &param);
        void condensedDistances();
        void hierarchicalClustering();
        void DBSCANclustering();
        void clustersVectorFun();
        float clusterDistFromGround(float x, float y, float z);
        Eigen::Vector3f regressCircle(const Eigen::VectorXf &xC, const Eigen::VectorXf &yC);
    public:
        PointcloudProcessing(std::string pathToConfigFile = "./lidarConfig.ini");

        PointcloudProcessing(int pclSize, std::string pathToConfigFile = "./lidarConfig.ini");

        PointcloudProcessing(Eigen::VectorXf *X, 
                             Eigen::VectorXf *Y, 
                             Eigen::VectorXf *Z, 
                             Eigen::Matrix<uint16_t, Eigen::Dynamic,1> *intensities,
                             std::string pathToConfigFile = "./lidarConfig.ini");

        PointcloudProcessing(std::unique_ptr<Eigen::VectorXf> &X, 
                             std::unique_ptr<Eigen::VectorXf> &Y, 
                             std::unique_ptr<Eigen::VectorXf> &Z, 
                             std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> &intensities,
                             std::string pathToConfigFile = "./lidarConfig.ini");

        void printPointcloudSize();
        void printSettings(); 
        
        Eigen::VectorXf getX();
        Eigen::VectorXf getY();
        Eigen::VectorXf getZ();
        Eigen::VectorXf getAzim();
        Eigen::VectorXf getR();
        Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> getIntensities();
        Eigen::VectorXi getClusters();
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getCart();

        bool filter();
        void calculatePartitionMatrix();
        void calculatePrototypePointsMatrix();
        void checkPrototypePointsMatrix();
        void groundLinesFit();
        void groundClassifier();
        bool filterGround();
        void nonGroundClustering();
        bool clusterClassifier(GNBC &nbc,
                               Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> &conePos);
        int pipeline(std::unique_ptr<Eigen::VectorXf> &X, 
                                                                           std::unique_ptr<Eigen::VectorXf> &Y, 
                                                                           std::unique_ptr<Eigen::VectorXf> &Z,
                                                                           std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> &intensities, 
                                                                           GNBC &nbc,
                                                                           Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> &conePos);
};

#endif // POINTCLOUD_PROCESSING_H_INCLUDED
