#include <mjpeg_to_ros/mjpeg_to_ros.h>

MJPEGToRosPublisher::MJPEGToRosPublisher(const std::string &node_name)
    : Node(node_name), running_(true) {

    // 비디오 URL 초기화
    video_urls_ = {
        "http://192.168.1.48:8081", // 카메라 1
        "http://192.168.1.48:8082", // 카메라 2
        "http://192.168.1.48:8083", // 카메라 3
        "http://192.168.1.48:8084"  // 카메라 4
    };

    // 퍼블리셔 초기화
    publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>("/Image/msfs/cam_1", 10));
    publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>("/Image/msfs/cam_2", 10));
    publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>("/Image/msfs/cam_3", 10));
    publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>("/Image/msfs/cam_4", 10));

    // 각 스트림에 대해 스레드 시작
    for (size_t i = 0; i < video_urls_.size(); ++i) {
        threads_.emplace_back(
            &MJPEGToRosPublisher::startStream, this, video_urls_[i], publishers_[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Started MJPEGToRosPublisher with %lu streams.", video_urls_.size());
}

MJPEGToRosPublisher::~MJPEGToRosPublisher() {
    stopThreads();
}

void MJPEGToRosPublisher::getParams() {
    this->declare_parameter<double>("example_param", 0.0);
}

void MJPEGToRosPublisher::startStream(
    const std::string &url,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher) {

    cv::VideoCapture cap;
    if (!cap.open(url)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open stream: %s", url.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Streaming from: %s", url.c_str());

    while (rclcpp::ok() && running_) {
        cv::Mat frame;
        cap >> frame;  // 프레임 읽기

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame received from: %s", url.c_str());
            continue;
        }
        int height = frame.rows;
        int width = frame.cols;
        int top  = height * 0.05;
        int bottom  = height * 0.95;
        int left  = width * 0.05;
        int right  = width * 0.95;
        cv::Rect roi(left, top, right-left, bottom - top);
        cv::Mat resized = frame(roi);

        // OpenCV Mat -> ROS 메시지 변환
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized).toImageMsg();
        publisher->publish(*image_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Published frame from: %s", url.c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(33));  // 약 30 FPS 유지
    }

    cap.release();
    RCLCPP_INFO(this->get_logger(), "Stopped streaming from: %s", url.c_str());
}

void MJPEGToRosPublisher::stopThreads() {
    running_ = false;  // 스레드 중지 신호 보내기
    for (auto &thread : threads_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
}
