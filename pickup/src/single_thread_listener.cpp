#include "ros/ros.h"

#include <pickup_msg/PickupObj.h>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// 接收到订阅的消息后，会进入消息回调函数
void Callback(const pickup_msg::PickupObj::ConstPtr& msg)
{
//    for (size_t i = 0; i < msg->boxes.size(); ++i) printf("%f ", msg->boxes[i]);
    for (size_t i = 0; i < msg->labels.size(); ++i) printf("%d ", msg->labels[i]);

    std::vector<cv::Rect2f> boxes;
    uint32_t boxesNum = msg->boxes.size()/4;
    for (size_t i = 0; i < boxesNum; ++i){
        boxes.push_back(cv::Rect2f( cv::Point2f(msg->boxes[4*i], msg->boxes[4*i+1]), cv::Point2f(msg->boxes[4*i+2], msg->boxes[4*i+3])));
    }

    std::vector<uint32_t> labels = msg->labels;

    if(boxesNum == msg->labels.size()) {
        printf("\ndetected %d objects\n", boxesNum);
    } else printf("erro\n");

}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "listener");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为PickupObjMsg的topic，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("PickupObjMsg", 1, Callback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}