# Lidar Vision Pipeline 

A package containing C++ library files and a test executable that implement a ground removal algorithm and provides cone detection (WIP) for our vehicle.  

![](https://i.imgur.com/bcOJtMK.png)

#### Prerequisites  
- Boost (Usually every Unix OS comes preinstalled with Boost. If not, install every submodule with `sudo apt-get install libboost-all-dev`)
- [Eigen](https://eigen.tuxfamily.org/)
- CMake 3.10 minimum
- ROS2
- [turtle_common](https://gitlab.com/aristurtle/dv/turtle_common)
- [turtle_interfaces](https://gitlab.com/aristurtle/dv/turtle_interfaces)

#### Installation instructions  
Clone the repo to your ROS2 workspace src folder using `git clone https://gitlab.com/aristurtle/dv/turtle_lidar.git --recursive` and built it like a normal ROS2 package using `colcon build`
