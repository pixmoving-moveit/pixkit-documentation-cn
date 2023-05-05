# 传感器标定介绍

## 概述
在自动驾驶系统中，传感器标定是一个非常关键的过程，对于确保自动驾驶系统的安全和性能至关重要。自动驾驶系统通常需要多种传感器来获取车辆周围的环境信息，包括摄像头、激光雷达、毫米波雷达等。这些传感器的标定可以帮助自动驾驶系统准确地感知和理解车辆周围的环境。

具体来说，传感器标定在自动驾驶系统中的作用包括：

1. 精确定位：通过标定，可以准确地确定不同传感器之间的位置、朝向和相对距离，从而实现更精确的定位和导航。
2. 优化障碍物检测：标定可以帮助自动驾驶系统准确地感知和理解车辆周围的障碍物，从而避免与其他车辆或物体发生碰撞。
3. 提高车道保持能力：标定可以提高自动驾驶系统的车道保持能力，使车辆能够更加准确地识别和跟踪车道。
4. 改进预测能力：标定可以提高自动驾驶系统的预测能力，使其能够更加准确地预测其他车辆和行人的行动和意图，从而提高车辆的安全性和驾驶体验。


## 传感器标定内容

- [相机内参标定](./camera%E5%86%85%E5%8F%82%E6%A0%87%E5%AE%9A.md)
- [激光雷达到相机的外参标定](./LiDAR-camera%E6%A0%87%E5%AE%9A.md)
- [IMU标定](./IMU%E6%A0%87%E5%AE%9A.md)
- [LiDAR-IMU标定](./LiDAR-IMU%E6%A0%87%E5%AE%9A.md)

## 传感器硬件

- lidar 
    - [RS-Helios-16P](https://www.robosense.cn/rslidar/RS-Helios)
- camera
    - [森云SG2-OX03CC-5200-GMSL2F-H120](https://www.sensing-world.com/productinfo/913484.html)
- 组合导航
    - [华测CHC® CGI-410](https://www.huace.cn/product/product_show/467)

## NEXT
现在，您已经完成`传感器标定介绍`，接下来可以开始:

- [标定工具的安装](./%E6%A0%87%E5%AE%9A%E5%B7%A5%E5%85%B7%E5%AE%89%E8%A3%85.md) 

## 引用

- [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration)
- [image_pipeline](https://github.com/ros-perception/image_pipeline)
- [calibration_tools](https://github.com/autocore-ai/calibration_tools)
- [camera_calibration](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [imu_utils](https://github.com/gaowenliang/imu_utils)
- [code_utils](https://github.com/gaowenliang/code_utils)

