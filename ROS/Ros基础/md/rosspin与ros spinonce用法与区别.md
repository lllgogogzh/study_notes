# rosspin与ros spinonce用法与区别

### 1、ros::spin()

程序达到rosspin之前，按照一系列规定，设定一系列话题订阅者，这些订阅者就等待话题消息，直到ros::spin()执行后，订阅者开始订阅消息，**进入回调函数**。如果最开始的时候，没有话题进来，那么程序会堵塞在ros::spin()函数，并且执行此函数后不会再进入主程序执行其他语句

```c++
int main(int argc,char** argv)
{

    ros::init(argc, argv, "message_filter_node");  // 相当于给一个命名空间
    ros::Time::init();
    ros::NodeHandle n;
    Scan2 scan;  // 默认构造


    Odom_calib.Set_data_len(12000);
    Odom_calib.set_data_zero();
    ros::spin(); // 主程序到这里往下不再进行，等待话题进来回调就行


    return 0;
}
```

### 2、ros::spinOnce()

这个函数与rosspin区别就是，执行完rosspinOnce后会继续往下执行，只调用一次

### 3、多个回调函数

多个回调函数情况下，假设：

- 第一个发布：10Hz
- 第二个发布：50Hz

​	使用ros::spin时，可以接收到所有消息

​	使用ros::spinOnce时，若队列为1的情况下，随时都接收最新数据：以10Hz运行ros::spinOnce ，则50Hz的发布会丢失(中间)数据；以50Hz运行则不丢失中间数据。

​	举个例子：

#### 多线程发布：

#### 回调函数数据：























