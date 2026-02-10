#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

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
#define DF_IMAGE_PORTNUM    5001 // 이미지 수신 포트
#define DF_UDP_SERVER_ADDR  "192.168.1.83" // MSFS 2024 PC
#define DF_UDP_CLIENT_ADDR  "192.168.1.70" // Autonomy PC

struct Flight_STATE
{
	char    title[256];
	double  kohlsmann;
	double  altitude;
	double  latitude;
	double  longitude;
	double  pitch;
	double  bank;
	double  heading;
	double	heading_true;
	double  velocityX;
	double  velocityY;
	double  velodityZ;
	double  temperature;
	double  airpressure;
	double  airdensity;
	double  windvelocity;
	double  winddirection;
	double  windx;
	double  windy;
	double  windz;
	double  airspeed;
	double	groundspeed;
	double  vertical_speed;
	double  latitude_vel;
	double  longitude_vel;
	double  altitude_vel;
	double  latitude_acc;
	double  longitude_acc;
	double  altitude_acc;
	double  pitch_vel;
	double  roll_vel;
	double  heading_vel;
	char atcMessage[256];
};

struct SW_STATE
{
	char	title[256];
	int	engineR;
	int	engineL;
	int	starter;
	int	alternator;
	int	battery;
	int	avionics;
	int	parking_brake;
	int	fuel_pump;
	int	beacon;
	int	landing;
	int	taxing;
	int	navigation;
	int	strobe;
	int	pitot_heat;
	double	aileron_pos;
	double  elevator_pos;

	int	fuel_tank_selector;
	float	trim_position;
	double	throttle;
	double	prop;
	double	mixture;
	int	flaps;
	int	gear;
	float	left_brake = 0.0;
	float	right_brake = 0.0;
};

#pragma pack (push, 1)
struct RX_message_data {
    uint32_t vehID;
    uint8_t emergencyStop;
    double desSpeed;
    double desSteer;
};
#pragma pack(pop)

class UDPReceiver {
public:
    UDPReceiver();
    ~UDPReceiver();
    void receiveData();
	bool receiveImageData(cv::Mat &image, std::string &windowName);
    // void start();

	Flight_STATE getFlightState();
	SW_STATE getSwitchState();
	
	Flight_STATE m_flight_state;
	SW_STATE m_sw_state;

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

#endif // UDP_RECEIVER_H
