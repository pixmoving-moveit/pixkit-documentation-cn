# IMU标定

## 数据准备

### imu

- apt-get install libceres-dev
- apt-get install libdw-dev
#### (1)imu内参标定

- 推荐使用频率200hz以上的imu，在使用之前需要标定imu的内外参，使用[imu_utils](https://github.com/gaowenliang/imu_utils)开源工具进行标定，需要有ROS[code_utils](https://github.com/gaowenliang/code_utils)工具环境。

- 录制2小时sensor_msgs/Imu数据.

- 录制好的数据已200倍速率播放

- 需更具实际录制时间修改max_time_min的参数

  ![image-20210901162433731](/Users/jinbeng/Library/Application Support/typora-user-images/image-20210901162433731.png)

- 标定好的内参存在imu_utils/data/[vendor_of_imu]_imu_param.yaml文件中的`gyr_n` `gyr_w` `acc_n` `acc_w`参数替换到LIO-SAM的param.yaml文件中的

  Imu_utils/data/xsens_imu_param.yaml

```
Gyr:
    unit: " rad/s"
    avg-axis:
    gyr_n: 1.5059072284923697e-03
    gyr_w: 4.3430855283551206e-05
    x-axis:
    gyr_n: 1.6901233770452774e-03
    gyr_w: 5.0850707578827144e-05
    y-axis:
    gyr_n: 1.3392742394140514e-03
    gyr_w: 3.7654685426892668e-05
    z-axis:
    gyr_n: 1.4883240690177805e-03
    gyr_w: 4.1787172844933785e-05
Acc:
    unit: " m/s^2"
    avg-axis:
    acc_n: 5.9215155351791055e-03
    acc_w: 1.3379378640306186e-04
    x-axis:
    acc_n: 6.0017230453598448e-03
    acc_w: 1.0726720420556991e-04
    y-axis:
    acc_n: 6.7689914243794181e-03
    acc_w: 1.6961241589651517e-04
    z-axis:
    acc_n: 4.9938321357980535e-03
    acc_w: 1.2450173910710051e-04
```

Lio_sam/config/param.yaml

```
# IMU Settings
imuAccNoise: 5.9215155351791055e-03
imuGyrNoise: 1.5059072284923697e-03
imuAccBiasN: 1.3379378640306186e-04
imuGyrBiasN: 4.3430855283551206e-05
```

#### (2)imu外参标定

使用[lidar_aglin](https://github.com/miracle629/lidar_align)开源工具进行标定，需要对imu的数据进行积分获取transform，录制sensor_msgs/Pointcloud2与/sensor_msgs/Imu数据，bag路径放入lidar_aglin.launch的参数中

获取标定数据后替换掉lio_sam/config/param.yam中的imu外参参数



### LiDAR准备

* 需使用1.6.1的[velodyne](https://github.com/ros-drivers/velodyne)驱动，需要点云里的time字段来去畸变。



