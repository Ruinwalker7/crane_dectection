# 相机驱动模块

## 1. 介绍

相机驱动模块包含两个ROS2节点，分别是`rm_camera_driver_node`和`video_player`。

- `rm_camera_driver_node`：相机驱动节点，负责相机的初始化、参数配置、图像采集、图像发布等工作。目前支持大恒水星相机
- `video_player`：视频播放节点，负责播放视频文件。

## 2. 发布话题

### "/image_raw" (sensor_msgs/msg/Image) 相机采集的图像

### "/camera_info" (sensor_msgs/msg/CameraInfo) 相机的参数信息

