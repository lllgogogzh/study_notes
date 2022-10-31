# ROS navigation Stack 安装

github源码：https://github.com/ros-planning/navigation/tree/melodic-devel

## 一、安装

​	我们从github源码：https://github.com/ros-planning/navigation/tree/melodic-devel中直接下载即可。放入工作空间，再`catkin_make`即可

​	如果报错：

```sh
tf2_sensor_msgsConfig.cmake
tf2_sensor_msgs-config.cmake
```

我们安装包即可：

```sh
sudo apt-get install ros-melodic-tf2-sensor-msgs
```

注意，melodic根据情况换成自己的ros版本。