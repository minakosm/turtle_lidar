# Lidar Vision Pipeline 

A package containing C++ library files and a test executable that implement a ground removal algorithm and provides cone detection (WIP) for our vehicle.  

![](https://i.imgur.com/bcOJtMK.png)

#### Prerequisites  
- Boost (Usually every Unix OS comes preinstalled with Boost)
- [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz)
- CMake 3.10 minimum


#### Installation instructions  
1. git clone https://gitlab.com/aristurtle/dv/lidar-vision-pipeline.git --branch develop --recursive
2. mkdir build
3. cd build
4. cmake ..
5. make

You can test the library using points from example/lidar.txt file by running:  
./TestExample  
in the build directory. You can change the pipeline parameters by editing config.ini (no need to rebuild the whole project)  
  
#### TODO   
ROS Wrapper  
Cone color detection from cone clusters
