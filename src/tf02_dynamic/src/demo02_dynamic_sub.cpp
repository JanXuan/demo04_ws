#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"//创建订阅对象
#include "tf2_ros/buffer.h"//与上面结合使用，缓存订阅的东西
#include "geometry_msgs/PointStamped.h"//坐标点
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"//转换后要用这个格式发送
/*
    订阅发布的坐标系相对关系，传入一个坐标点调用tf实现
    步骤：
        1.包含头文件
        2.编码、初始化、NodeHandle
        3.创建订阅对象订阅坐标系相对关系
        4.组织坐标点数据
        5.转换算法，调用tf内置功能
        6.输出
*/
int main(int argc, char *argv[])
{
    /* code */
    //2.编码、初始化、NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_sub");
    ros::NodeHandle nh;
    //3.创建订阅对象订阅坐标系相对关系
        //创建buffer缓存
    tf2_ros::Buffer buffer;
        //创建订阅对象可以将订阅数据存到buffer中
    tf2_ros::TransformListener listener(buffer);
    //4.组织坐标点数据
    geometry_msgs::PointStamped ps;
    //参考坐标系是子坐标系乌龟
    ps.header.frame_id = "turtle1";
    //每一句都有时间戳而动态变换buffer值多导致Time::now()获取时间与buffer时间太大报错
    ps.header.stamp = ros::Time(0.0);
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    //方式一：在while前添加休眠订阅到坐标与坐标的相对关系
    //ros::Duration(2).sleep();
    //5.转换算法，调用tf内置功能
    ros::Rate rate(10);
    while (ros::ok())
    {
        geometry_msgs::PointStamped ps_out;
        try
        {
            ps_out = buffer.transform(ps,"world");
        //6.输出
            ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f),参考坐标系:%s",
            ps_out.point.x,ps_out.point.y,ps_out.point.z,ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常消息：%s",e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}