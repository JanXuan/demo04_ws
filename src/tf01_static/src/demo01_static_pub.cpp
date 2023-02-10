#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
/*
    需求：该节点需要发布两个坐标系的相对关系
    步骤：
        1.包含头文件
        2.初始化(编码，节点，NodeHandle)
        3.创建发布对象
        4.组织被发布的消息
        5.发布数据
        6.spin()
*/
int main(int argc, char *argv[])
{
    /* code */
    //2.初始化(编码，节点，NodeHandle)
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub");
    ros::NodeHandle nh;
    //3.创建发布对象
    tf2_ros::StaticTransformBroadcaster pub;
    //4.组织被发布的消息
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "base_link";//相对坐标系中被参考坐标系，主坐标系
    tfs.child_frame_id = "laser";//雷达
    tfs.transform.translation.x = 0.2;
    tfs.transform.translation.y = 0.0;
    tfs.transform.translation.z = 0.5;
    //需要根据欧拉角转换成四元数，用四元数的头文件
    tf2::Quaternion qtn;//创建四元数对象
    //给此对象设置欧拉角即可将其转换为四元数
    qtn.setRPY(0,0,0);//欧拉角单位是弧度
    tfs.transform.rotation.w = qtn.getW();
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();

    //5.发布数据
    pub.sendTransform(tfs);
    //6.spin()
    ros::spin();
    return 0;
}
