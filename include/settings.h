#ifndef SETTINGS_H
#define SETTINGS_H

#define NUM_OF_THREADS 2

#include <ostream>

struct BaseFilter {
    bool filterEnabled, filterAzim, filterRad, filterZ;
    float maxAzim, minAzim, maxRad, minRad, maxZ, minZ;

    BaseFilter(std::string filepath = "./config.ini");
    BaseFilter(int &segments, int &bins, std::string filepath = "./config.ini");
};

struct GroundFilter {
    float mMax, mSmall, bMax, bMin, maxRmse, dPrev, dGround;

    GroundFilter(std::string filepath = "./config.ini");
};

struct ClusterSettings {
    int clusteringMethod, DBminPts;
    float hierClusterDist, DBepsilon;

    ClusterSettings(std::string filepath = "./config.ini");
};

struct ClassifierSettings {
    int ignoreClusterPointsLow, ignoreClusterPointsHigh, regressCircleMaxIter;
    bool reconstructCluster, useOriginalClusterCircle;
    float r, zMin, zMax, regressCircleDiffThreshold;
    
    ClassifierSettings(std::string filepath = "./config.ini");
};

std::ostream &operator<<(std::ostream &os, BaseFilter const &m);
std::ostream &operator<<(std::ostream &os, GroundFilter const &m);
std::ostream &operator<<(std::ostream &os, ClusterSettings const &m);
std::ostream &operator<<(std::ostream &os, ClassifierSettings const &m);

#endif // SETTINGS_H_INCLUDED
