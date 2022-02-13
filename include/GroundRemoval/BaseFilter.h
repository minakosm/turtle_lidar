#ifndef BASE_FILTER_H
#define BASE_FILTER_H

#include <string>
#include <ostream>

class BaseFilter
{
private:
    float minAzim, maxAzim, minR, maxR, minZ, maxZ;
    bool enableBaseFilter;

public:
    BaseFilter(std::string filepath="./lidarConfig.ini");
    
    bool getEnableBaseFilter() {
        return enableBaseFilter;
    }
    
    float getMaxAzim() {
        return maxAzim;
    }

    float getMinAzim() {
        return minAzim;
    }

    float getMinR() {
        return minR;
    }

    float getMaxR() {
        return maxR;
    }

    float getMinZ() {
        return minZ;
    }

    float getMaxZ() {
        return maxZ;
    }

    
};

std::ostream &operator<<(std::ostream os, BaseFilter m);


#endif // BASE_FILTER_H_INCLUDED