#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
/*
    需求：
    1.换算出turtle1相对于turtle2的关系
    2.计算角速度和线速度并发布
*/
int main(int argc, char *argv[])
{
    //2.编码、初始化、NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub");
    ros::NodeHandle nh;
    //3.创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);
    //要发布速度消息要创建发布对象
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",100);
    //4.编写逻辑
    ros::Rate rate(10);
    while (ros::ok())
    {
        try
        {
            //计算son1与son2相对关系
            /*
                A相对于B的坐标系关系
                lookupTransform函数
                参数1：目标坐标系B
                参数2：源坐标系A
                参数3：ros::Time(0)，取间隔最短的两个坐标关系帧计算相对关系
                返回值：geometry_msgs::TransformStamped类型，源相对于目标坐标系的相对关系
            */
            geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
            //ROS_INFO("turtle1相对于turtle2的信息：父级:%s，子级:%s，偏移量:(%.2f，%.2f，%.2f)",
            //                    son1Toson2.header.frame_id.c_str(),//turtle2
            //                    son1Toson2.child_frame_id.c_str(),//turtle1
            //                    son1Toson2.transform.translation.x,
            //                    son1Toson2.transform.translation.y,
            //                    son1Toson2.transform.translation.z);
            //2.根据相对关系计算并组织速度消息
            geometry_msgs::Twist twist;
            /*
                组织速度，只需要线速度的x,和角速度的z
                线速度 x = 系数 * sqrt(x^2 + y^2)
                角速度 y = 系数 * arctan(y/x)
            */
            twist.linear.x = 0.5 * sqrt(pow(son1Toson2.transform.translation.x,2) + pow(son1Toson2.transform.translation.y,2));
            twist.angular.z = 4 * atan2(son1Toson2.transform.translation.y,son1Toson2.transform.translation.x);
            //3.发布
            pub.publish(twist);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("错误提示：%s",e.what());
        }
        rate.sleep();
        ros::spinOnce();
        
    }
    
    //5.spinOnce()

    return 0;
}