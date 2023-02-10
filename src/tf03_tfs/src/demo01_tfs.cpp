#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
/*
    订阅方实现：
        1.转换son1与son2坐标关系
        2.计算son1中目标点在son2中坐标值
    流程：
        1.包含头文件
        2.编码、初始化、NodeHandle
        3.创建订阅对象
        4.编写逻辑
        5.spinOnce()
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
    //4.编写逻辑
    //创建坐标点
    geometry_msgs::PointStamped psAtson1;
    psAtson1.header.stamp = ros::Time::now();
    psAtson1.header.frame_id = "son1";
    psAtson1.point.x = 1.0;
    psAtson1.point.y = 2.0;
    psAtson1.point.z = 3.0;
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
            geometry_msgs::TransformStamped son1Toson2 = buffer.lookupTransform("son2","son1",ros::Time(0));
            ROS_INFO("son1相对于son2的信息：父级:%s，子级:%s，偏移量:(%.2f，%.2f，%.2f)",
                                son1Toson2.header.frame_id.c_str(),
                                son1Toson2.child_frame_id.c_str(),
                                son1Toson2.transform.translation.x,
                                son1Toson2.transform.translation.y,
                                son1Toson2.transform.translation.z);
            //son1中坐标点在son2中坐标值
            geometry_msgs::PointStamped psAtson2 = buffer.transform(psAtson1,"son2");
            ROS_INFO("坐标点在son2中的值(%.2f,%.2f,%.2f)",
                                        psAtson2.point.x,
                                        psAtson2.point.y,
                                        psAtson2.point.z
                                        );
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
