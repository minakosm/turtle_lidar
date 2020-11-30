#include "LidarDriver.h"
#include "PointcloudProcessing.h"
#include "os1.h"

int main() {
    LidarDriver lidar;
    lidar.run_driver();
    return 0;
}