#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "rosbag/view.h"
/*
    需求：使用rosbag读取磁盘上的bag文件
        1.导包
        2.初始化
        3.创建rosbag对象
        4.以读方式打开文件流
        5.读数据
        6.关闭文件流
*/
int main(int argc, char *argv[])
{
    //2.初始化
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"bag_read");
    ros::NodeHandle nh;
    //3.创建rosbag对象
    rosbag::Bag bag;
    //4.以读方式打开文件流
    bag.open("hello.bag",rosbag::BagMode::Read);
    //5.读数据
    //取出话题、时间戳、消息
    //可以先获取消息的集合再迭代取出字符
    for(auto &&m : rosbag::View(bag))
    {
        //解析
        std::string topic = m.getTopic();
        ros::Time time = m.getTime();
        std_msgs::StringPtr p = m.instantiate<std_msgs::String>();
        ROS_INFO("解析的内容,话题:%s,时间戳:%.2f,消息值:%s",
                                    topic.c_str(),
                                    time.toSec(),
                                    p->data.c_str());
    }
    //6.关闭文件流
    bag.close();
    return 0;
}
