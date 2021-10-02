#ifndef POINTCLOUD_PROCESSING_H
#define POINTCLOUD_PROCESSING_H

#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <memory>
#include <fstream>

#include "settings.h"
#include "lines.h"
#include "hpdbscan.h"
#include "bayes.h"
#include "utils.h"

class PointcloudProcessing {
    private:
        int segments;   // Number of segments the PCL is divided
        int bins;       // Number of bins each segment is divided
        float minAzim, maxAzim, minRad, maxRad, shiftAngle, shiftRad;   // PCL characteristics after BaseFilter has been applied
        
        BaseFilter baseFilter;                  // BaseFilter settings
        GroundFilter groundFilter;              // Ground removal filter settings
        ClusterSettings clusterSettings;        // Cluster-related settings
        ClassifierSettings classifierSettings;  // Cone classifier settings

        std::unique_ptr<Eigen::VectorXf> xBuffer, yBuffer, zBuffer, azimBuffer, rBuffer;    // Buffer variables used from the beginning of the pipeline up to applying the BaseFilter
        std::unique_ptr<Eigen::VectorXf> x, y, z, azim, r;                                  // Variables that are the result of the BaseFilter on corresponding buffer variables
        std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> intensities, intensitiesBuffer, intensitiesFiltered;    // Cartesian variables that are the result of GroundFilter on x,y,z unique_ptr variables
        std::unique_ptr<Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor>> cart;    // Variables used to hold intensity values (one buffer variable to be used up to BaseFiltering, one variable to be used up to GroundFiltering and one for the rest of pipepline)

        Eigen::Matrix <uint8_t, Eigen::Dynamic, 2, Eigen::RowMajor> partitionMatrix;                // (number_of_points_after_BaseFilter x 2) Matrix that holds information about the segment and bin each point belongs to
        Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> prototypePointsMatrix; // (segments x bins) Matrix that holds the index of the prototype point of each segment-bin combination
        Lines lines;                                            // struct variable that holds information about the lines regressed in each segment
        Eigen::Array <bool, Eigen::Dynamic, 1> groundArray;     // (number_of_points_after_BaseFilter x 1) Vector that contains true or false corresponding to a point being a ground point or a non-ground point
        Eigen::VectorXd condensedClusterDistances;              // condensed vector that contains distances between non-ground points used for hierarchical clustering (number_of_non_ground_points * (number_of_non_ground_points - 1) / 2))
        HPDBSCAN hpdbscanClusterer;                               // Object containing methods for Highly Parallel DBSCAN clustering
        Eigen::VectorXi clusters;                               // Vector that contains the label of the cluster a specific point belongs to (number_of_non_ground_points x 1)
        int numberOfClusters;                                   // Number of clusters
        Eigen::Array <bool, Eigen::Dynamic, 1> coneClusters;
        Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> clusterPositions;
        
        void polarInit();
        bool filterPoints(const Eigen::Array <bool, Eigen::Dynamic, 1> &logicalVector);
        Eigen::VectorXf getLine(const Eigen::VectorXf &properties);
        Eigen::VectorXf updateLine(float x, float y, const Eigen::VectorXf &properties);
        SegmentLines segmentGroundLinesFit(int segment);
        inline float distPointFromLine(float x, float y, const Eigen::VectorXf &param);
        void condensedDistances();
        void hierarchicalClustering();
        void DBSCANclustering();
        void HPDBSCANclustering();
        void clustersVectorFun();
        float clusterDistFromGround(float x, float y, float z);
        Eigen::Vector3f regressCircle(const Eigen::VectorXf &xC, const Eigen::VectorXf &yC);
    public:
        PointcloudProcessing(std::string pathToConfigFile = "./lidarConfig.ini");
        PointcloudProcessing(int pclSize, std::string pathToConfigFile = "./lidarConfig.ini");

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
        Eigen::VectorXi getClusters();
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getCart();
        int getNonGroundPoints();
        BaseFilter getBaseFilterSettings();
        GroundFilter getGroundFilterSettings();
        ClusterSettings getClusterSettings();
        ClassifierSettings getClassifierSettings();

        bool filter();
        void calculatePartitionMatrix();
        void calculatePrototypePointsMatrix();
        void checkPrototypePointsMatrix();
        void groundLinesFit();
        void groundClassifier();
        bool filterGround();
        void nonGroundClustering();
        bool clusterClassifier(GNBC &nbc,
                               Eigen::Matrix <float, Eigen::Dynamic, 2, 
                               Eigen::RowMajor> &conePos);
        int pipeline(std::unique_ptr<Eigen::VectorXf> &X, 
                     std::unique_ptr<Eigen::VectorXf> &Y, 
                     std::unique_ptr<Eigen::VectorXf> &Z,
                     std::unique_ptr<Eigen::Matrix<uint16_t, Eigen::Dynamic, 1>> &intensities, 
                     GNBC &nbc,
                     Eigen::Matrix <float, Eigen::Dynamic, 2, Eigen::RowMajor> &conePos,
                     int maxPointsProcessing = 10000, int timeoutProcessing = 90);
};

#endif // POINTCLOUD_PROCESSING_H_INCLUDED
