# AMCL

参考：[AMCL算法原理讲解_RedGlass_lee的博客-CSDN博客_amcl定位原理](https://blog.csdn.net/qq_33742147/article/details/105489516)

[slam 学习之 AMCL 概念与原理分析_ppipp1109的博客-CSDN博客_amcl定位原理](https://blog.csdn.net/p942005405/article/details/108569970)

## 一、概述

​	按照一定规律生成粒子，读取里程计odom信息，更新粒子位置并加入噪声。将激光雷达信息作为观测，判断测量数据发生的可能性，以这个可能性来更新粒子的权重。下次增加粒子大多数会在这些高权重粒子附近添加。最后，对收敛的粒子位置取平均即可得到基于地图的全局位置。

​	**注意：amcl定位需要给一个较好的初始化位置，否则机器人后续运动过程中，粒子收敛速度会比较慢。**

​	**amcl是在地图中的全局定位，其读取轮式里程计信息，并利用激光雷达信息对机器人进行定位。算法主要思想为贝叶斯推断，把轮式里程计信息视为先验，激光信息为测量。**

​	*目前较好的定位方式有：轮式里程计+激光雷达；视觉+imu；激光雷达+imu，其中，初始化过程可以采用识别世界坐标系中二维码的关于机器人的相对位姿来实现。*
