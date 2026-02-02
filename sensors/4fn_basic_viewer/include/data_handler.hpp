#ifndef DATA_HANDLER_H_
#define DATA_HANDLER_H_

#include <string>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "socket.hpp"

using namespace std::chrono_literals;

class DataHandler : public rclcpp::Node
{
    public :
        DataHandler() : Node("radar_node")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_pcl2", 10);
            timer_ = this->create_wall_timer(
                        2ms, std::bind(&DataHandler::timer_callback, this));
            
            init();
        }
        ~DataHandler() { }

        const double pi = acos(-1);

    private :
        void timer_callback()
        {
            if(receive()) {
                publish();
            }
        }

        void    init();
        int8_t  receive();
        int8_t  publish();
        float   polar2x(float range, float azimuth, float elevation);
        float   polar2y(float range, float azimuth, float elevation);
        float   polar2z(float range, float azimuth, float elevation);

        rclcpp::TimerBase::SharedPtr                                timer_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

        std::array<point_data_t, MAX_NUM_POINTS_PER_FRAME>          r_data;
        sensor_msgs::msg::PointCloud2                               cloud2;
        uint32_t                                                    frame_number, nPoints;
        Socket                                                      m_socket;
        int                                                         z_offset;
};

#endif  // DATA_HANDLER_H_