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
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    //方式一：在while前添加休眠订阅到坐标与坐标的相对关系
    //ros::Duration(2).sleep();
    //5.转换算法，调用tf内置功能
    ros::Rate rate(10);
    while (ros::ok())
    {
        /* code */
        //核心代码转换ps--相对于底盘的坐标
        /*
            调用了buffer里的transform函数，参数1：被转换坐标点
                                        参数2：目标坐标系
                                        返回：目标坐标系下的坐标值
            头文件必须包含"tf2_geometry_msgs/tf2_geometry_msgs.h"
            如果订阅方找不到base_link是因为发布方的坐标关系还没来
            //方式一：在while前添加休眠订阅到坐标与坐标的相对关系
                //ros::Duration(2).sleep();
            //方式2：异常处理
            try
        {
            ps_out = buffer.transform(ps,"base_link");
            ROS_INFO("转换后的坐标值：(%.2f,%.2f,%.2f),参考坐标系:%s",
            ps_out.point.x,ps_out.point.y,ps_out.point.z,ps_out.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常消息：%s",e.what());
        }
        */
        geometry_msgs::PointStamped ps_out;
        try
        {
            ps_out = buffer.transform(ps,"base_link");
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
