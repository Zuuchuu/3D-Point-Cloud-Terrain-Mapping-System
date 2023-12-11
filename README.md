# 3D Point Cloud Terrain Mapping System

In this 3D terrain mapping system, data collection from sensors during the UAV survey is done initially and further offline processing of the data is done to build a 3D map. The implementation of this research uses mainly Robotic Operating System (ROS) based on Raspberry Pi 4 running Ubuntu 18.04. Therefore, the software libraries used for the implementation are divided into two tasks: Data collection and Offline data processing.

![My Image](image/BlockDiagram.png)

Data Collection
---------------
The data collection involves collecting data from Velodyne VLP-16 LIDAR, IMU AdaFruit BNO055, Emlid Reach RTK GNSS and Raspberry Pi 4. The sensors data is collected using the ROS bag tool. The system needs to be setup with appropriate ROS drivers to receive data from sensors into ROS architecture. The below figure illustrates architectural diagram of sensor setup used for data collection

![My Image](image/BlockDiagram.png)

Velodyne LIDAR
---------------

Download Velodyne ROS driver using:
```
git clone git@github.com:ros-drivers/velodyne.git
```
To run the Velodyne sensor ROS driver:
```
roslaunch velodyne\_pointcloud VLP16_points.launch
```

Emlid Reach RTK
---------------

The NMEA sentences from the Emlid Reach and GNSS UTC can be logged using the nmea_navsat driver. This driver is forked from 
```
https://github.com/ros-drivers/nmea_navsat_driver
```

To run the driver:
```
roslaunch nmea_navsat_driver nmea_serial_driver.launch
```

IMU BNO055
---------------

Download BNO055 ROS driver using:
```
git clone git@github.com:RoboticArts/ros_imu_bno055.git
```
To run the Velodyne sensor ROS driver:
```
roslaunch ros_imu_bno055 imu.launch serial_port:=/dev/ttyUSB0 operation_mode:=NDOF
```
