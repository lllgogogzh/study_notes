# ROS中的坐标管理系统

## 一、机器人中的坐标变换

实现小海龟跟随。

```shell
sudo apt-get install ros-kinetic-turtle-tf
roslanuch turtle_tf turtle_tf_demo.launch
rosrun turtlesim turtle_teleop_key
rosrun tf view_frames
```

查询tf树

```shell
rosrun tf view_frame
#或
rosrun tf view_frames xxx1 xxx2
#xxx1 xxx2为坐标系
#或运行rviz
rosrun rviz rviz-d 'rospack find turtle_tf'/rviz/turtle_rviz.rviz
```



## 二、tf坐标系广播与监听的编程实现

### ·创建功能包

```
catkin_create_pkg test_tf roscpp rospy tf turtlesim
```

### ·创建tf广播器代码

```c++
//产生tf数据，并计算、发布turtle2的速度指令
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    //解算出来的是turtle1/2对world坐标系的相对位置
    //创建tf广播器
    static tf::TransformBroadcaster br;
    //初始化tf数据
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x,msg->y,0.0));//设置平移关系
    tf::Quaternion q;//四元数
    q.setRPY(0,0,msg->theta);
    transform.setRotation(q);//设置旋转关系
    
    //广播world与海龟坐标系之间的tf数据
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world",turtle_name));
}

int main(int argc,char **argv)
{
    //初始化ROS节点
    ros::init(argc,argv,"my_tf_broadcaster");
    //输入参数作为海龟的名字
    if(argc!=2)
    {
        ROS_ERROR("need turtle name as argument!");
        return -1;
    }
    
    turtle_name = argv[1];
    
    //订阅海龟姿态话题
    ros::NodeHandle node;
    ros::Subscriber sub=node.subscribe(turtle_name+"/pose",10,&poseCallback);
    ros::spin();
}
```

main函数中：输入海龟名字是为了运行两遍，把turtle1和turtle2的坐标系都广播出去。

### ·创建tf监听器代码

定义TF监听器

查找坐标变换

```c++
//监听坐标系
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math>
#include <turtlesim/Spawn.h>
#include <tf/transform_listener.h>
int main(int argc,char **argv)
{
    //初始化ROS节点
    ros::init(argc,argv,"my_tf_listener");
    //创建节点句柄
    ros::NodeHandle node;
    
    //请求产生turtle2
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle=node.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);
    
    //创建发布turtle2速度控制指令的发布者
    ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("/turlte2/cmd_vel",10);
    //创建tf监听器
    tf::TransformListener listener;
    
    ros::Rate rate(10);
    while(node.ok())
    {
        //获取turtle1与turtle2坐标系之间的tf数据
        tf::StampedTransform transform;
        try
        {  
            listener.waitForTransform("/turtle2","/turtle1",ros::Time(0),                			ros::Duration(3.0));
         	listener.lookupTransform("/turtle2","/turtle1",ros::Time(0),transform);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        //根据turtle1和2坐标系间的位置关系，发布turtle2的速度控制指令
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z=4.0*atan2(transform.getOrigin().y(),transform.getOrigin().x());
        vel_msg.linear.x=0.5*sqrt(pow(transform.getOrigin().x(),2)+
                                 pow(transform.getOrigin().y(),2));
        turtle_vel.publish(vel_msg);
        rate.sleep();
    }
    return 0;
}
```

### ·编写CMakeLists.txt文件

```cmake
add_executable(turtle_tf_broadcaster src/turtle_tf_broadcaster.cpp)
target_link_libraries(turtle_tf_broadcaster ${catkin_LIBRARIES})

add_executable(turtle_tf_listener src/turtle_tf_listener.cpp)
target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
```

### ·运行程序

```
catkin_make
source devel/setup.bash
roscore
rosrun turtlesim turtlesim_node
rosrun test_tf turtle_tf_broadcaster __name:=turtle1_tf_broadcaster /turtle1
rosrun test_tf turtle_tf_broadcaster __name:=turtle2_tf_broadcaster /turtle2
rosrun test_tf turtle_tf_listener
rosrun turtlesim turtle_teleop_key
```

### ·附：编写的launch文件

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="sim" output="screen"/>
	<node pkg="turtlesim" type="turtle_teleop_key" name="key"/>
	<node pkg="test_pkg" type="velocity_publisher" name="vel_publisher"/>
    <node pkg="test_tf" type="turtle_tf_broadcaster" args="/turtle1" name="turtle1_tf_broadcaster" output="screen"/>
    <node pkg="test_tf" type="turtle_tf_broadcaster" args="/turtle2" name="turtle2_tf_broadcaster" output="screen"/>
	<node pkg="test_tf" type="turtle_tf_listener" name="listener_123"/>
</launch>
```

### ·附：程序运行图片

![](..\image_ros1\13.png)

### ·附：ros节点关系图

![](..\image_ros1\11.png)

