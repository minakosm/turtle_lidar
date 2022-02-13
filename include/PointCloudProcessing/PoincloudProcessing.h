#ifndef POINTCLOUD_PROCESSING_H
#define POINTCLOUD_PROCESSING_H

#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <memory>
#include <fstream>

#include "settings.h"
#include "lines.h"
//#include "hpdbscan.h"
//#include "bayes.h"


class PointcloudProcessing {
    private:
        int segments; 
        int bins;
        float minAzim, maxAzim, minRad, maxRad, shiftAngle, shiftRad;

        BaseFilter baseFilter;
        GroundFilter groundFilter;


        std::unique_ptr<Eigen::VectorXf> xBuffer, yBuffer, zBuffer, azimBuffer, rBuffer;
        std::unique_ptr<Eigen::VectorXf> x, y, z, azim, r;
        std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic,1>> intensities, intensitiesBuffer, intensitiesFiltered;
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor>> cart;

        Eigen::Matrix <uint8_t, Eigen::Dynamic, 2, Eigen::RowMajor> partitionMatrix;
        Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prototypePointsMatrix;
        Lines lines;
        Eigen::Array <bool, Eigen::Dynamic, 1> groundArray;
        Eigen::VectorXd condensedClusterDistances;
        
        //POLAR_INIT
        void polarInit();
        bool filterPoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector);
        Eigen::VectorXf getLine(const Eigen::VectorXf &properties);
        Eigen::VectorXf updateLine(float x, float y, const Eigen::VectorXf &properties);
        SegmentLines segmentGroundLinesFit(int segment);
        inline float distPointFromLine(float x, float y, const Eigen::VectorXf &param);
        void condensedDistances();

    public:
        PointcloudProcessing(std::string pathToConfigFile ="./lidarConfig.ini");
        PointcloudProcessing(int pclSize, std::string pathToConfigFile ="./lidarConfig.ini");

        void resizeCoordinates(int pclSize);
        void printPointcloudSize();
        void printSettings();

        // Getters
        Eigen::VectorXf getX();
        Eigen::VectorXf getY();
        Eigen::VectorXf getZ();
        Eigen::VectorXf getAzim();
        Eigen::VectorXf getR();
        Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> getIntensities();
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getCart();
        Eigen::Matrix<uint16_t, Eigen::Dynamic, 1> getIntensitiesFiltered();
        int getNonGroundPoints();
        BaseFilter getBaseFilterSettings();
        GroundFilter getGroundFilterSettings();

        bool filter();
        void calculatePartitionMatrix();
        void calculatePrototypePointsMatrix();
        void checkPrototypePointsMatrix();
        void groundLinesFit();
        void groundClassifier();
        bool filterGround();

        int pipeline(std::unique_ptr<Eigen::VectorXf> &X,
                     std::unique_ptr<Eigen::VectorXf> &Y,
                     std::unique_ptr<Eigen::VectorXf> &Z,
                     std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> &intensities,
                     Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> &conePos,
                     int maxPointsProcessing = 10000, int timeoutProcessing = 90);

};

#endif //POINTCLOUD_PROCESSING_H_INCLUDED