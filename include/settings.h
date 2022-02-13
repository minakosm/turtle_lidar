#ifndef SETTINGS_H
#define SETTINGS_H

#define NUM_OF_THREADS 2 

#include <ostream>

//BASE_FILTER
struct BaseFilter{
    bool filterEnabled, filterAzim, filterRad, filterZ;
    float maxAzim, minAzim, maxRad, minRad, maxZ, minZ;

    BaseFilter(std::string filepath = "./lidarConfig.ini");
    BaseFilter(int &segments, int &bins, std::string filepath = "./lidarConfig.ini");
};

//GROUND_FILTER
struct GroundFilter{
    float mMax, mSmall, bMax, bMin, maxRmse, dPrev, dGround;

    GroundFilter(std::string filepath = "./lidarConfig.ini");
};

//CLUSTER_SETTINGS

//CLASSIFIER SETTINGS

std::ostream &operator<<(std::ostream &os, BaseFilter const &m);
std::ostream &operator<<(std::ostream &os, GroundFilter const &m);

//std::ostream &operator<<(std::ostream &os, ClusterSettings const &m);
//std::ostream &operator<<(std::ostream &os, ClassifierSettings const &m);


#endif // SETTINGS_H_INCLUDED