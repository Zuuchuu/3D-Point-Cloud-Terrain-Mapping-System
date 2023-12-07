# 3D Point Cloud Terrain Mapping System

In this 3D terrain mapping system, data collection from sensors during the UAV survey is done initially and further offline processing of the data is done to build a 3D map. The implementation of this research uses mainly Robotic Operating System (ROS) based on Raspberry Pi 4 running Ubuntu 18.04. Therefore, the software libraries used for the implementation are divided into two tasks: Data collection and Offline data processing.

Data Collection
---------------
The data collection involves collecting data from Velodyne VLP-16 LIDAR, IMU AdaFruit BNO055, Emlid Reach RTK GNSS and Raspberry Pi 4. The sensors data is collected using the ROS bag tool. The system needs to be setup with appropriate ROS drivers to receive data from sensors into ROS architecture. The below figure illustrates architectural diagram of sensor setup used for data collection
