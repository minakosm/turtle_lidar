#include "GroundRemoval/BaseFilter.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp> 
#include <string>

BaseFilter::BaseFilter(std::string filepath){
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filepath, pt);

    enableBaseFilter = pt.get<bool>("BaseFilter.enableBaseFilter");

    minAzim = pt.get<float>("BaseFilter.minAzim");
    maxAzim = pt.get<float>("BaseFilter.maxAzim");

    minR = pt.get<float>("BaseFilter.minR");
    maxR = pt.get<float>("BaseFilter.maxR");

    minZ = pt.get<float>("BaseFilter.minZ");
    maxZ = pt.get<float>("BaseFilter.maxZ");
}

std::ostream &operator<<(std::ostream os, BaseFilter m){
    return os << "enableBaseFilter = " << m.getEnableBaseFilter()
              << "\nminAzim = " << m.getMinAzim()  
              << "\nmaxAzim = " << m.getMaxAzim()
              << "\nminR = " << m.getMinR()
              << "\nmaxR = " << m.getMaxR()
              << "\nminZ = " << m.getMinZ()
              << "\nmaxZ = " << m.getMaxZ();
}
