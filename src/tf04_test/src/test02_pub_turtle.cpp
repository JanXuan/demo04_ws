#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"//动态发布
#include "geometry_msgs/TransformStamped.h"//消息在这
#include "tf2/LinearMath/Quaternion.h"//欧拉角四元数转换
/*
    发布方：需要订阅乌龟的位姿信息，并转换成相对于世界坐标系的坐标关系并发布
    准备：
        rosrun turtlesim turtlesim_node
        rostopic info /turtle1/pose
        rosmsg info turtlesim/Pose
        话题：/turtle1/pose
        消息：/turtlesim/Pose
    步骤：
        1.包含头文件
        2.设置编码、初始化、NodeHandle
        3.创建订阅对象订阅turtle1/pose
        4.回调函数处理订阅消息，将位姿信息转换成坐标相对关系并发布
        5.spin()
*/
//声明变量接收传递的参数
std::string turtle_name;
void doPose(const turtlesim::Pose::ConstPtr& pose)
{
    //获取位姿信息转换成相对关系发布
    //创建发布对象static就可以每次调用不用又生成发布对象
    static tf2_ros::TransformBroadcaster pub;
    //组织被发布的数据
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = "world";
    ts.header.stamp = ros::Time::now();
    //关键点2：frame_id动态传入
    ts.child_frame_id = turtle_name;
    //坐标系偏移量
    ts.transform.translation.x = pose->x;
    ts.transform.translation.y = pose->y;
    ts.transform.translation.z = 0;
    //四元数乌龟只有偏航角，所以乌龟欧拉角为0 0 theta
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    //发布
    pub.sendTransform(ts);
}
int main(int argc, char *argv[])
{
    //2.设置编码、初始化、NodeHandle
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub");
    ros::NodeHandle nh;
    /*
        解析launch文件传入的args参数
    */
    if(argc != 2)
    {
        ROS_ERROR("请传入一个参数！");
        return 1;
    }
    else
    {
        turtle_name=argv[1];
    }
    
    //3.创建订阅对象订阅turtle1/pose
    //关键点1：订阅的话题名称，turtle1/turtle2要为动态传入

    ros::Subscriber sub = nh.subscribe(turtle_name + "/pose",100,doPose);
    //4.回调函数处理订阅消息，将位姿信息转换成坐标相对关系并发布

    //5.spin()
    ros::spin();
    return 0;
}
