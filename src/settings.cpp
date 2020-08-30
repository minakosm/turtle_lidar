#include "settings.h"

#include <ostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <string>

BaseFilter::BaseFilter() {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);

    filterEnabled = pt.get<bool>("BaseFilter.filterEnabled");
    filterAzim = pt.get<bool>("BaseFilter.filterAzim");
    filterRad = pt.get<bool>("BaseFilter.filterRad");
    filterZ = pt.get<bool>("BaseFilter.filterZ");

    maxAzim = pt.get<float>("BaseFilter.maxAzim");
    minAzim = pt.get<float>("BaseFilter.minAzim");
    maxRad = pt.get<float>("BaseFilter.maxRad"); 
    minRad = pt.get<float>("BaseFilter.minRad");
    maxZ = pt.get<float>("BaseFilter.maxZ");
    minZ = pt.get<float>("BaseFilter.minZ");
}

BaseFilter::BaseFilter(int &segments, int &bins) {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);

	segments = pt.get<int>("Parameters.segments");
	bins = pt.get<int>("Parameters.bins");

    filterEnabled = pt.get<bool>("BaseFilter.filterEnabled");
    filterAzim = pt.get<bool>("BaseFilter.filterAzim");
    filterRad = pt.get<bool>("BaseFilter.filterRad");
    filterZ = pt.get<bool>("BaseFilter.filterZ");

    maxAzim = pt.get<float>("BaseFilter.maxAzim");
    minAzim = pt.get<float>("BaseFilter.minAzim");
    maxRad = pt.get<float>("BaseFilter.maxRad"); 
    minRad = pt.get<float>("BaseFilter.minRad");
    maxZ = pt.get<float>("BaseFilter.maxZ");
    minZ = pt.get<float>("BaseFilter.minZ");
}

GroundFilter::GroundFilter() {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);

    mMax = pt.get<float>("GroundFilter.mMax");
    mSmall = pt.get<float>("GroundFilter.mSmall");
    bMax = pt.get<float>("GroundFilter.bMax");
    bMin = pt.get<float>("GroundFilter.bMin");
    maxRmse = pt.get<float>("GroundFilter.maxRmse");
    dPrev = pt.get<float>("GroundFilter.dPrev");
    dGround = pt.get<float>("GroundFilter.dGround");
}

ClusterSettings::ClusterSettings() {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);

    clusteringMethod = pt.get<int>("Cluster.clusteringMethod");
    DBminPts = pt.get<int>("Cluster.DBminPts");

    hierClusterDist = pt.get<float>("Cluster.hierClusterDist");
    DBepsilon = pt.get<float>("Cluster.DBepsilon");
}

ClassifierSettings::ClassifierSettings() {
    boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);

    ignoreClusterPointsLow = pt.get<int>("Classifier.ignoreClusterPointsLow");
    ignoreClusterPointsHigh = pt.get<int>("Classifier.ignoreClusterPointsHigh");
    regressCircleMaxIter = pt.get<int>("Classifier.regressCircleMaxIter");
    
    reconstructCluster = pt.get<bool>("Classifier.reconstructCluster");
    useOriginalClusterCircle = pt.get<bool>("Classifier.useOriginalClusterCircle");

    r = pt.get<float>("Classifier.r");
    zMin = pt.get<float>("Classifier.zMin");
    zMax = pt.get<float>("Classifier.zMax");
    furtherReconstructDist = pt.get<float>("Classifier.furtherReconstructDist");
    regressCircleDiffThreshold = pt.get<float>("Classifier.regressCircleDiffThreshold");
}

std::ostream &operator<<(std::ostream &os, BaseFilter const &m) { 
    return os << "filterEnabled = " << m.filterEnabled << "\nfilterAzim = "
              << m.filterAzim << "\nfilterRad = " << m.filterRad << "\nfilterZ = "
              << m.filterZ << "\nmaxAzim = " << m.maxAzim << "\nminAzim = "
              << m.minAzim << "\nmaxRad = " << m.maxRad << "\nminRad = "
              << m.minRad << "\nmaxZ = " << m.maxZ << "\nminZ = " << m.minZ 
              << std::endl << std::endl;
}

std::ostream &operator<<(std::ostream &os, GroundFilter const &m) { 
    return os << "mMax = " << m.mMax << "\nmSmall = " << m.mSmall << "\nbMax = " 
              << m.bMax << "\nbMin = " << m.bMin << "\nmaxRmse = " << m.maxRmse 
              << "\ndPrev = " << m.dPrev << "\ndGround = " << m.dGround 
              << std::endl << std::endl;
}

std::ostream &operator<<(std::ostream &os, ClusterSettings const &m) {
    if(m.clusteringMethod == 0)
        return os << "clusteringMethod = Hierarchical" << "\nhierClusterDist = " << m.hierClusterDist 
                  << std::endl << std::endl;
    else if(m.clusteringMethod == 1)
        return os << "clusteringMethod = DBSCAN\nDBepsilon = " << m.DBepsilon << "\nDBminPts = " << m.DBminPts
                  << std::endl << std::endl;
    return os << "Invalid clustering method chosen!\n";
    
}

std::ostream &operator<<(std::ostream &os, ClassifierSettings const &m) {
    return os << "ignoreClusterPointsLow = " << m.ignoreClusterPointsLow << "\nignoreClusterPointsHigh = " << m.ignoreClusterPointsHigh 
              << "\nregressCircleMaxIter = " << m.regressCircleMaxIter << "\nreconstructCluster = " << m.reconstructCluster 
              << "\nuseOriginalClusterCircle = " << m.useOriginalClusterCircle << "\nr = " << m.r << "\nzMin = " << m.zMin << "\nzMax = " << m.zMax
              << "\nfurtherReconstructDist = " << m.furtherReconstructDist << "\nregressCircleDiffThreshold = " << m.regressCircleDiffThreshold 
              << std::endl << std::endl;
}
