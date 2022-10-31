# ROS中用类实现节点的编写

## 一、用类实现发布者编写（简易版）

```c++
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;

class Test_Node
{
public:
    Test_Node(ros::NodeHandle &n);//construction function
    void publish();//publish function
private:
    ros::Publisher chatter_pub;//set the publisher as a member
};

Test_Node::Test_Node(ros::NodeHandle& n)//detail of the construction function
{
    chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10); //assign value to chatter_pub
}
void Test_Node::publish()//detail of function
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;
        chatter_pub.publish(vel_msg);
        ROS_INFO("Publish turtle velocity command[%0.2f m/s,%0.2f rad/s]",vel_msg.linear.x,vel_msg.angular.z);
        loop_rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"testcpp");
    ros::NodeHandle node;
    Test_Node TN(node);
    TN.publish();
    return 0;
}
```

```xml
<launch>
	<node pkg="turtlesim" type="turtlesim_node" name="turtle1"/>
	<node pkg="lei_cpp" type="1" name="vel_pub" output="screen"/>
</launch>
```

