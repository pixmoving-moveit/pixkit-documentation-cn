# LiDAR-LiDAR标定

## 概述
## 前提条件
- 完成了[标定工具安装](./%E6%A0%87%E5%AE%9A%E5%B7%A5%E5%85%B7%E5%AE%89%E8%A3%85.md)
- 准备硬件：
    - 顶部激光雷达[RS-Helios-16P]
    - 车头补盲雷达[RS-Bpearl]
    
## 开始标定
### step-2: 选择标定产场地
- 车头前方有大象立体物体

### step-1: 启动标定程序

> 观察rviz2，当白色点云和彩色点云，完全重合时标定结束

> - 白色点云: 是顶部雷达
> - 彩色点云: 是车头补盲雷达

```shell
./calibration_script/lidar2lidar/run_lidar2lidar.sh
```


