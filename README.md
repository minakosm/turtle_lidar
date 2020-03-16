# Lidar Vision Pipeline 

A package containing library files and a test executable that implement the ground removal algorithm for our vehicle

#### Prerequisites:
Boost (Usually every Unix OS comes preinstalled with Boost)
Eigen 3.3
CMake 3.10 minimum

#### Installation instructions:
mkdir build
cd build
cmake ..
make

You can test the library using points from example/lidar.txt file and running:
./TestExample
in the build directory

TODO:
Clustering
Cone detection
ROS Wrapper