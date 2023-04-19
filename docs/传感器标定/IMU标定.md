# IMU标定

## 概要

IMU（惯性测量单元）是自主驾驶汽车中重要的传感器之一，可以测量车辆在三维空间中的加速度和角速度信息。为了保证自主驾驶汽车的精确控制和定位，需要对IMU进行内参标定。内参标定的目的是精确定量测量设备的误差参数，包括加速度计和陀螺仪的偏置、比例因子、非正交性等参数。通过内参标定可以提高IMU的精度，从而提高自主驾驶汽车的定位精度和控制精度。

## 前提条件
- 完成了[标定工具安装](./%E6%A0%87%E5%AE%9A%E5%B7%A5%E5%85%B7%E5%AE%89%E8%A3%85.md)
- 准备硬件：
    - [华测CHC® CGI-410](https://www.huace.cn/product/product_show/467)

## 开始标定
## 数据准备
### step-1: 检测摄像头是否联通工控机
### imu

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


## NEXT
现在，您已经完成`camera内参标定`，[LiDAR-IMU标定](./LiDAR-IMU%E6%A0%87%E5%AE%9A.md)