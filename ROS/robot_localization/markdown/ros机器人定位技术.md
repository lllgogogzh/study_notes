# ROS机器人定位技术

参考：[robot_localization - ROS Wiki](http://wiki.ros.org/robot_localization)

[机器人定位技术：robot_localization - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/121622661)

## 一、概述

参考：[robot_localization wiki — robot_localization 2.6.11 documentation (ros.org)](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

| word           | translation |
| -------------- | ----------- |
| implementation | 执行        |
|                |             |
|                |             |

​	**robot_localization是基于卡尔曼滤波在ROS系统上比较成熟、应用比较广泛的一个机器人动态定位软件包**

*From official documents*

​	所有`robot_localization`状态估计节点有共同的特点：

1. 可以融合任意数量的传感器信息。例如：机器人上有多个IMU传感器，那么程序可以一并支持它们。
2. 支持多种ROS消息类型。包括：`nav_masgs/Odometry`、`sensor_msgs/Imu`、`geometry_msgs/PoseWithConvarianceStamped`、`geometry_msgs/TwistWithCovarianceStamped`。
3. 自定义化的传感器预处理。如果给定的传感器消息包含您不想包含在状态估计中的数据，`robot_localization` 中的状态估计节点允许您基于每个传感器排除该数据。
4. 连续估计。可以理解为实时估计。
5. 估计15维机器人状态：位置xyz，欧拉角rpy，速度vx,vy,vz，角速度和加速度。

## 二、EKF非线性卡尔曼滤波

[SLAM学习笔记----卡尔曼滤波详解_YOULANSHENGMENG的博客-CSDN博客_卡尔曼滤波](https://blog.csdn.net/YOULANSHENGMENG/article/details/124929480)

[详解卡尔曼滤波原理_engineerlixl的博客-CSDN博客_卡尔曼滤波](https://blog.csdn.net/u010720661/article/details/63253509?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-1-63253509-blog-124929480.pc_relevant_aa&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2~default~CTRLIST~default-1-63253509-blog-124929480.pc_relevant_aa)

[How a Kalman filter works, in pictures | Bzarg](http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

### 1、KF卡尔曼滤波

​	首先复习一下卡尔曼滤波。

#### a.定义状态量

​	我们用一个简单的例子来分析。估计机器人的位置以及速度。
$$
\boldsymbol{x}=
\begin{bmatrix}
p\\v
\end{bmatrix}
$$
我们不知道实际的位置和速度，其可能有多种组合，如下图所示。但其中一些的可能性要大于其他部分(高斯分布)。

![widthscale=0.6](..\img\gauss_0 (1).png)

其中，上图表示的速度与位置没有相关性，**这意味着由其中一个变量的状态无法推测出另一个变量的可能值**。其可能的点的位置也较多。

​	下面给出速度和位置是相关的情况，如下图所示

![](..\img\gauss_3.png)

这意味着两个待估计变量有关。例如速度与位置，速度快则位置变化快，速度慢则位置变化慢，**这非常重要，因为其给我们带来更多的信息：其中一个测量值告诉了我们其他变量的可能值**。这就是卡尔曼滤波的目的，尽可能地在包含不确定性的测量数据中提取更多的信息。

​	这种相关性用协方差矩阵来表示。从一个高斯分布变成了另一个高斯分布。

![](..\img\gauss_2.png)

#### b.利用矩阵描述问题(预测部分)

​	我们要估计k时刻的最有估计值以及其协方差矩阵，如下式所示：
$$
\hat{x}_k=
\begin{bmatrix}
p\\v
\end{bmatrix},
P_k=
\begin{bmatrix}
\Sigma_{pp}&\Sigma_{pv}\\
\Sigma_{vp}&\Sigma_{vv}
\end{bmatrix}
$$
我们要根据上一时刻的状态(k-1)预测当前状态(k)，根据模型预测(或根据其他传感器)。**我们不知道对下一状态的所有预测中，哪一个是真是的，但预测函数并不在乎。它对所有的可能性进行预测，并给出新的高斯分布。**

![](..\img\gauss_7.jpg)

​	我们可以用矩阵Fk来表示这个过程（这就是预测矩阵，可以根据机器人运动学模型，或者传感器模型等）：

![](..\img\gauss_8.jpg)

​	它将我们原始估计中的每个点都移到了一个新的预测位置，若原始估计正确，那么这个新的预测位置就是系统下一步会移动到的位置。在本例子中，我们用匀(加)速运动模型来处理：
$$
p_k=p_{k-1}+\Delta tv_{k-1}\\
v_k=v_{k-1}
$$
现更新协方差矩阵（线性递推状态量的协方差）：
$$
Cov(F_{k}x_{k-1})=F_kP_{k-1}F_k^T
$$

#### c.外部控制量

​	可能存在外部因素对系统造成影响，例如外部控制：导航软件可能发出一个让轮子转或停的指令，我们是知道这些指令的，用uk向量表示，我们将其加到我们的预测方程中做修正。

​	假设由于油门的设置或控制命令，我们知道了期望的加速度a，根据基本运动学方程可以得到：
$$
p_k=p_{k-1}+\Delta t v_{k-1}+\frac{1}{2}a\Delta t^2\\
v_k=v_{k-1}+a \Delta t
$$
写成矩阵形式则为：
$$
\check x_k=F_k \hat{x}_{k-1}+B_ku_k
$$
Bk为控制矩阵，uk为控制向量。

​	**我们再考虑，预测以及外部控制量并不是100%准确的**

#### d.外部干扰

​	每个预测值都是高斯分布，如下图所示

![](..\img\gauss_9.jpg)

原始估计中的每个状态变量更新到新的状态后，仍然服从高斯分布。其实可以认为是输入量带入的不确定性。这样，产生了具有不同协方差的新的高斯分布(均值相同)。

![](..\img\gauss_10b.jpg)

我们通过添加Qk得到扩展的协方差。

#### e.预测的完整表达

$$
\check x_k=F_k \hat{x}_{k-1}+B_ku_k\\
\check P_k=F_k\hat P_{k-1}F_k^T+Q_k
$$

#### f.用测量来修正估计值

​	我们可能会有多个传感器来测量系统的当前状态，可能是一个测量速度，一个测量位置，或间接地告诉了我们一些状态信息。

![](..\img\gauss_12-624x287.jpg)

如上图所示，传感器的数据单位和尺度(或一些间接数据)可能与我们想要估计的状态的单位和尺度不一样，其中可以用一个线性关系来进行转换。我们用矩阵Hk来表示传感器数据的转换。*具体例子可以考虑针孔相机模型*。

​	我们来表示传感器模型(现在还没考虑传感器的误差)：
$$
z_{\mu_{expected}} = H_k\check x_k\\
z_{\Sigma_{expected}} = H_k\check P_k H_k^T\\
$$
​	**我们现在考虑传感器的误差：**

![](..\img\gauss_14.jpg)



![](..\img\gauss_11.jpg)

这样的高斯分布，测量均值不变，加一个方差即可。我们将这个不确定性用协方差Rk表示，该分布的均值就是我们读取到的传感器数据，记为zk。

#### g.融合预测与测量

​	**现在我们有了两个高斯分布：一个是在预测值附近，一个是在传感器读数附近。**

![](..\img\gauss_4.jpg)

如上图所示，**我们想在预测值(粉色)和传感器测量值(绿色)**之间找到最优解。我们直接对这两个高斯分布相乘，也就是融合，得到了一个新的高斯分布，这个高斯分布就是我们想要的最佳估计值。如下图所示：

![](..\img\gauss_6a.png)

卡尔曼滤波整个大体过程就是这样，现在对卡尔曼滤波进行数学推导：

#### h. 数学推导(融合高斯分布)

​	*先推导一维，再根据一维推演到高维，但这个推演并没有严格的数学推导。因此，我们再利用贝叶斯公式推导高维。*

![](..\img\20170318174028764.jpg)

##### 一维推导

​	两个高斯分布相乘，我们先看指数项：
$$
-\frac{1}{2}[\frac{(x-\mu_0)^2}{\sigma_0^2}+\frac{(x-\mu_1)^2}{\sigma_1^2
}]\\=
-\frac{1}{2}
\frac{(\sigma_0^2+\sigma_1^2)x^2-2(\mu_0\sigma_1^2+\mu_1\sigma_0^2)x+\sigma_1^2\mu_0^2+\sigma_0^2\mu_1^2}
{\sigma_0^2\sigma_1^2}
$$
我们之后对x进行完全平方合并即可，多出的常数项补到前面系数，再进行归一化。

最后我们得到新的高斯分布的均值和方差为：
$$
\mu_{new}=\mu_0+\frac{\sigma_0(\mu_1-\mu_0)}{\sigma_0^2+\sigma_1^2}\\
\sigma_{new}^2=\sigma_0^2-\frac{\sigma^4_0}{\sigma_0^2+\sigma_1^2}
$$
我们把上式中，相同部分用k表示：
$$
k=\frac{\sigma^2_0}{\sigma_0^2+\sigma_1^2}\\
\mu_{new}=\mu_0+k(\mu_1-\mu_0)\\
\sigma_{new}^2=\sigma_0^2-k\sigma_0^2
$$

##### 推演到高维

​	*注意：这里的推演没有数学依据，只是非常直观。*

​	我们把上式写为矩阵形式：
$$
\boldsymbol{K}=\boldsymbol{\Sigma_0(\Sigma_0+\Sigma_1)^{-1}}\\
\boldsymbol{\mu_{new}}=\boldsymbol{\mu_0}+\boldsymbol{K}
(\boldsymbol{\mu_1}-\boldsymbol{\mu_0})\\
\boldsymbol{\Sigma_{new}}=\boldsymbol{\Sigma_{0}}-\boldsymbol{K}
\boldsymbol{\Sigma_{0}}
$$

##### 数学推导高维

​	我们严格按照数学依据推导高维高斯分布的情况。

#### i. 总结



![](..\img\kalflow.png)





## 三、源码学习

[robot_localization 源码解析（1）ekf_localization_node_Zack_Liu的博客-CSDN博客](https://blog.csdn.net/zack_liu/article/details/112344346)

