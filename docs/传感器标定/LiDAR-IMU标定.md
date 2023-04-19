# LiDAR-IMU标定

## (2)imu外参标定

使用[lidar_aglin](https://github.com/miracle629/lidar_align)开源工具进行标定，需要对imu的数据进行积分获取transform，录制sensor_msgs/Pointcloud2与/sensor_msgs/Imu数据，bag路径放入lidar_aglin.launch的参数中

获取标定数据后替换掉lio_sam/config/param.yam中的imu外参参数



### LiDAR准备

* 需使用1.6.1的[velodyne](https://github.com/ros-drivers/velodyne)驱动，需要点云里的time字段来去畸变。