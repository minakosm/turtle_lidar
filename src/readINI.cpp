#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "readINI.h"

void readINIfile(float bounds[], float groundParams[], int &segments, int &bins) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("../config.ini", pt);
	segments = pt.get<int>("Parameters.segments");
	bins = pt.get<int>("Parameters.bins");
	bounds[0] = pt.get<float>("BaseFilters.maxAzim");
	bounds[1] = pt.get<float>("BaseFilters.minAzim");
	bounds[2] = pt.get<float>("BaseFilters.maxRad");
	bounds[3] = pt.get<float>("BaseFilters.minRad");
	bounds[4] = pt.get<float>("BaseFilters.maxZ");
	bounds[5] = pt.get<float>("BaseFilters.minZ");
	groundParams[0] = pt.get<float>("GroundFitting.Tm");
	groundParams[1] = pt.get<float>("GroundFitting.Tmsmall"); 
	groundParams[2] = pt.get<float>("GroundFitting.Tbmax"); 
	groundParams[3] = pt.get<float>("GroundFitting.Tbmin"); 
	groundParams[4] = pt.get<float>("GroundFitting.Trmse"); 
	groundParams[5] = pt.get<float>("GroundFitting.Tdprev"); 
}
