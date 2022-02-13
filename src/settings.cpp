#include <ostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <string>

#include "settings.h"

BaseFilter::BaseFilter(std::string filepath){
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filepath, pt);

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

BaseFilter::BaseFilter(int &segments, int &bins, std::string filepath){
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filepath, pt);

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

GroundFilter::GroundFilter(std::string filepath){
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filepath, pt);

    mMax = pt.get<float>("GroundFilter.mMax");
    mSmall = pt.get<float>("GroundFilter.mSmall");
    bMax = pt.get<float>("GroundFilter.bMax");
    bMin = pt.get<float>("GroundFilter.bMin");
    maxRmse = pt.get<float>("GroundFilter.maxRmse");
    dPrev = pt.get<float>("GroundFilter.dPrev");
    dGround = pt.get<float>("GroundFilter.dGround");
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


