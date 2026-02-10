#ifndef MJPEG_TO_ROS_PUB_H
#define MJPEG_TO_ROS_PUB_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <atomic>

class MJPEGToRosPublisher : public rclcpp::Node {
public:
    MJPEGToRosPublisher(const std::string &node_name);
    ~MJPEGToRosPublisher();

private:
    void getParams();
    void startStream(const std::string &url, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher);
    void stopThreads();

private:
    std::vector<std::thread> threads_;
    std::vector<std::string> video_urls_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
    std::atomic<bool> running_;
};

#endif // MJPEG_TO_ROS_PUB_H
