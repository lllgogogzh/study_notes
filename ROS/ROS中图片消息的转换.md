# ROS中图片消息的转换

## 一、正常节点回调函数

*使用C++类实现*

由于ROS图片消息和OpenCV图片消息的格式不一样，需要进行一次转换才可以，因此我们需要以下程序(讲解在注释中)。

```c++
#include "myslam/CommonInclude.h"
#include "myslam/Frame.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
using namespace std;

class FrameRead
{
    public:
        FrameRead();//构造函数
        ~FrameRead();//析构函数
    	//回调函数
        void color_img_callback(const sensor_msgs::ImageConstPtr& msg);
    private:    
        ros::NodeHandle nh;//节点句柄
        
        ros::Subscriber color_img_sub;//订阅者：color图
        ros::Subscriber depth_img_sub;//订阅者：depth图
};

FrameRead::FrameRead()
{
    //构造函数的实现
    //从topic：“/rgb”读取消息，回调函数为：FrameRead::color_img_callback
    color_img_sub = nh.subscribe("/rgb",1,&FrameRead::color_img_callback,this);
}

FrameRead::~FrameRead()
{
    //析构函数：关闭所有opencv打开的图片窗口
    cv::destroyAllWindows();
}

void FrameRead::color_img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //回调函数实现
    //图片转换
    cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr
    try
    {
      //若成功捕获到消息，那么对ROS类型的图片消息进行转换，转换为OpenCV的格式
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      //若本次捕获消息失败，则catch error ，输出错误信息并且返回，进行下一次消息捕获。
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	//显示图像，cv_ptr->image为OpenCV格式的图像，可以直接显示
    imshow("1",cv_ptr->image);
    cv::waitKey(50);    
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"runtestnode");//节点初始化
    FrameRead imgRead;//初始化一个FrameRead类对象
    ros::spin();//开始进入回调函数循环
    return 0;
}
```

cmakelist文件：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(myslam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#executable
#Test for Test.cpp
add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})

#runtest for runtest.cpp
add_executable(runtest src/runtest.cpp)
target_link_libraries(runtest ${catkin_LIBRARIES} ${OpenCV_LIBS})

#imagefilter
add_executable(imagefilter src/imagefilter.cpp)
target_link_libraries(imagefilter ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

## 二、*同时间戳捕获所有传感器数据(message_filters)

*ROS消息过滤器(message_filters)*

http://wiki.ros.org/message_filters

### 1、概述

​	message_filter是roscpp和rospy的一个应用库，它集中了一些滤波算法中常用的一些消息。当一个消息到来，在之后的一个时间点，该消息可能被返回或者不返回，将这样的一个过程或者容器理解为消息滤波器。

​	例子：时间同步器(time synchronizer)，它可以从不同的源获取消息，仅当时间同步时将其输出。

### 2、使用

​	我们在例子中使用registerCallback()，功能：注册多个回调函数，此函数返回类型为message_filters::Connection。

​	我们先定义订阅者(Subscriber)，这里的订阅者提供信息源(就是普通的订阅者，获取信息)。用法如下：

```c++
message_filters::Subscriber<std_msgs::UInt32> sub(nh,"myTopic",1);
sub.registerCallback(myCallback);
//上述写法与下述写法等价
ros::Subscriber sub = nh.subscribe("myTopic",1,myCallback);
```

***使用时间同步器Time Synchronizer**

```c++
message_filters::Subscriber<Image> image_sub(nh,"image",1);
message_filters::Subscriber<CameraInfo> info_sub(nh,"camera_info",1);
TimeSynchronizer<Image,CameraInfo> sync(image_sub,info_sub,10);
sync.registerCallback(boost::bind(&callback,_1,_2));
```

​	定义两个订阅者，这两个订阅者的消息必须有相同的时间戳才可有输出。

### 3、代码以及分析

分析在注释中

```c++
#include "myslam/CommonInclude.h"
#include "myslam/Frame.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
using namespace std;

class ImageGrabber
{
    //图像读取类
    public:
    	//自定义的图像读取回调函数
        void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    protected:
};

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    //此处操作见第一讲
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("color",cv_ptrRGB->image);

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    imshow("depth",cv_ptrD->image);
    cv::waitKey(0);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "RGBD");//初始化节点
    ros::start();

    ImageGrabber igb;//声明对象
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/rgb", 1);//第一个订阅者(color)
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth", 1);//第二个订阅者(depth)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);//定义时间同步器(含义有待理解)
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));//注册回调函数(类中的回调函数都要指向对象本身)

    ros::spin();
    ros::shutdown();

    return 0;
}
```

