#ifndef IMAGE_RECEIVER_COMM_H
#define IMAGE_RECEIVER_COMM_H

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <string>

// Serial communication
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cmath>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>


#define DF_UDP_BUFFER_SIZE  4096
#define DF_UDP_PORTNUM      50001

#define DF_UDP_IMAGE_BUFFER_SIZE  8192
#define DF_IMAGE_PORTNUM    7357 // 이미지 수신 포트
#define DF_UDP_SERVER_ADDR  "10.1.25.145" // MSFS 2024 PC
#define DF_UDP_CLIENT_ADDR  "192.168.225.2" // Autonomy PC


#pragma pack (push, 1)
struct RX_message_data {
    uint32_t vehID;
    uint8_t emergencyStop;
    double desSpeed;
    double desSteer;
};
#pragma pack(pop)

class ImageReceiver {
public:
    ImageReceiver();
    ~ImageReceiver();
    void receiveData();
	bool receiveImageData(cv::Mat &image, std::string &windowName);
    // void start();
	bool receiveUdpImageOnce(cv::Mat &image);


private:
    int Socket;
    int imageSocket;
    struct sockaddr_in MyAddr;
    struct sockaddr_in imageAddr;

    char RXBuffer[DF_UDP_BUFFER_SIZE];

    void parseReceivedData(const std::string &data);
    void processFlightData(const std::string &data);
    void processSwitchData(const std::string &data);
	// void processImage(const std::vector<uchar>& buffer);
	void processImage(const std::vector<uchar>& buffer, const std::string& windowName);

	std::mutex state_mutex; // Mutex for protecting shared states

};

#endif // IMAGE_RECEIVER_COMM_H
