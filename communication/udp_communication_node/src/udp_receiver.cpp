#include <udp_communication/udp_receiver.h>
#include <chrono>
#include <iomanip> // setprecision을 사용하기 위해 필요

using namespace std::chrono_literals;
using namespace cv;
#define INFINITE 999999

UDPReceiver::UDPReceiver() 
{
  std::cout << "Initializing UDP Receiver..." << std::endl;

  // UDP 소켓 생성
  Socket = socket(PF_INET, SOCK_DGRAM, 0);
  if (Socket == -1) {
      std::cerr << "Socket creation failed!" << std::endl;
  }

  // UDP 서버 주소 설정
  memset(&MyAddr, 0, sizeof(MyAddr));
  MyAddr.sin_family = PF_INET;
  MyAddr.sin_port = htons(DF_UDP_PORTNUM);
  MyAddr.sin_addr.s_addr = inet_addr(DF_UDP_CLIENT_ADDR);

  if (bind(Socket, (struct sockaddr *)&MyAddr, sizeof(MyAddr)) != 0) {
      std::cerr << "Bind failed!" << std::endl;
  }

  // 이미지 데이터 소켓 생성
  imageSocket = socket(PF_INET, SOCK_DGRAM, 0);
  if (imageSocket == -1) {
      std::cerr << "Image socket creation failed!" << std::endl;
  }

  memset(&imageAddr, 0, sizeof(imageAddr));
  imageAddr.sin_family = PF_INET;
  imageAddr.sin_port = htons(DF_IMAGE_PORTNUM);
  imageAddr.sin_addr.s_addr = inet_addr(DF_UDP_CLIENT_ADDR);

  if (bind(imageSocket, (struct sockaddr*)&imageAddr, sizeof(imageAddr)) != 0) {
      std::cerr << "Image socket bind failed!" << std::endl;
  }

};

UDPReceiver::~UDPReceiver() {
    close(Socket);
    close(imageSocket);

}

// void UDPReceiver::start() {
//     std::thread flightThread(&UDPReceiver::receiveData, this);
//     std::thread imageThread(&UDPReceiver::receiveImageData, this);

//     flightThread.join();
//     imageThread.join();
// }

void UDPReceiver::receiveData() {
    socklen_t addrLen = sizeof(MyAddr);
    // while (true) {
    memset(RXBuffer, 0, sizeof(RXBuffer));
    int recvLen = recvfrom(Socket, RXBuffer, sizeof(RXBuffer), 0, (struct sockaddr *)&MyAddr, &addrLen);
    
    if (recvLen > 0) {
        std::string receivedData(RXBuffer, recvLen);
        parseReceivedData(receivedData);
    } else {
        std::cerr << "No data received or timeout." << std::endl;
    }
    // }
}

void UDPReceiver::parseReceivedData(const std::string &data) {
    if (data.find("HEADER=FLIGHT") == 0) {
        processFlightData(data);
    } else if (data.find("HEADER=SWITCH") == 0) {
        processSwitchData(data);
    } 
    // else if (data.find("HEADER=RX") == 0) {
    //     processRXMessageData(data);
    // } 
    else {
        std::cerr << "Unknown data format received." << std::endl;
    }
}

void UDPReceiver::processFlightData(const std::string &data) {
    Flight_STATE flight = {0}; // 구조체 초기화
    sscanf(data.c_str(), 
           "HEADER=FLIGHT LATITUDE=%lf LONGITUDE=%lf ALTITUDE=%lf KOHLSMANN=%lf PITCH=%lf ROLL=%lf HEADING=%lf HEADING_TRUE=%lf "
           "VELOCITY_X=%lf VELOCITY_Y=%lf VELOCITY_Z=%lf TEMPERATURE=%lf AIRPRESSURE=%lf AIRDENSITY=%lf WIND_VELOCITY=%lf "
           "WIND_DIRECTION=%lf WIND_X=%lf WIND_Y=%lf WIND_Z=%lf AIRSPEED=%lf GROUNDSPEED=%lf VERTICAL_SPEED=%lf "
           "LATITUDE_VEL=%lf LONGITUDE_VEL=%lf ALTITUDE_VEL=%lf LATITUDE_ACC=%lf LONGITUDE_ACC=%lf ALTITUDE_ACC=%lf "
           "PITCH_VEL=%lf ROLL_VEL=%lf HEADING_VEL=%lf",
           &flight.latitude, &flight.longitude, &flight.altitude, &flight.kohlsmann, 
           &flight.pitch, &flight.bank, &flight.heading, &flight.heading_true, 
           &flight.velocityX, &flight.velocityY, &flight.velodityZ, &flight.temperature, 
           &flight.airpressure, &flight.airdensity, &flight.windvelocity, &flight.winddirection, 
           &flight.windx, &flight.windy, &flight.windz, &flight.airspeed, &flight.groundspeed, 
           &flight.vertical_speed, &flight.latitude_vel, &flight.longitude_vel, &flight.altitude_vel, 
           &flight.latitude_acc, &flight.longitude_acc, &flight.altitude_acc, 
           &flight.pitch_vel, &flight.roll_vel, &flight.heading_vel);

    // 디버깅 출력
    // std::cout << std::fixed << std::setprecision(8)
    //           << "[FLIGHT] LAT=" << flight.latitude 
    //           << " LON=" << flight.longitude 
    //           << " ALT=" << flight.altitude 
    //           << " HEADING=" << flight.heading
    //           << " PITCH=" << flight.pitch
    //           << " ROLL=" << flight.bank
    //           << " AIRSPEED=" << flight.airspeed
    //           << " GNDSPD=" << flight.groundspeed
    //           << " WIND_VELOCITY=" << flight.windvelocity
    //           << " WIND_DIRECTION=" << flight.winddirection << std::endl;
    {
      std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
      m_flight_state = flight;
    }
	
}
void UDPReceiver::processSwitchData(const std::string &data) {
    SW_STATE sw = {0}; // 구조체 초기화
    sscanf(data.c_str(), 
           "HEADER=SWITCH ENGINE_R=%d ENGINE_L=%d STARTER=%d ALTERNATOR=%d BATTERY=%d AVIONICS=%d "
           "PARKING_BRAKE=%d FUEL_PUMP=%d BEACON=%d LANDING=%d TAXING=%d NAVIGATION=%d STROBE=%d PITOT_HEAT=%d "
           "AILERON_POS=%lf ELEVATOR_POS=%lf FUEL_TANK_SELECTOR=%d TRIM_POSITION=%f THROTTLE=%lf PROP=%lf MIXTURE=%lf "
           "FLAPS=%d GEAR=%d LEFT_BRAKE=%f RIGHT_BRAKE=%f",
           &sw.engineR, &sw.engineL, &sw.starter, &sw.alternator, &sw.battery, &sw.avionics, 
           &sw.parking_brake, &sw.fuel_pump, &sw.beacon, &sw.landing, &sw.taxing, &sw.navigation, 
           &sw.strobe, &sw.pitot_heat, &sw.aileron_pos, &sw.elevator_pos, &sw.fuel_tank_selector, 
           &sw.trim_position, &sw.throttle, &sw.prop, &sw.mixture, &sw.flaps, &sw.gear, 
           &sw.left_brake, &sw.right_brake);

    // 디버깅 출력
    // std::cout << "[SWITCH] ENGINE_R=" << sw.engineR 
    //           << " ENGINE_L=" << sw.engineL 
    //           << " STARTER=" << sw.starter
    //           << " PARKING_BRAKE=" << sw.parking_brake
    //           << " AILERON_POS=" << sw.aileron_pos
    //           << " ELEVATOR_POS=" << sw.elevator_pos
    //           << " THROTTLE=" << sw.throttle
    //           << " FLAPS=" << sw.flaps
    //           << " LEFT_BRAKE=" << sw.left_brake 
    //           << " RIGHT_BRAKE=" << sw.right_brake << std::endl;
    {
      std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
      m_sw_state = sw;
    }
}
bool UDPReceiver::receiveImageData(cv::Mat &image, std::string &windowName) {
    socklen_t addrLen = sizeof(imageAddr);
    char packet[DF_UDP_BUFFER_SIZE];
    std::vector<uchar> buffer;                  // 이미지 데이터를 저장할 버퍼
    std::string currentWindowName;              // 현재 수신 중인 윈도우 이름
    bool collectingData = false;                // 데이터 수집 중인지 확인

    while (true) {  // 하나의 완전한 이미지를 수신할 때까지 반복
        int recvLen = recvfrom(imageSocket, packet, DF_UDP_BUFFER_SIZE, 0, (struct sockaddr*)&imageAddr, &addrLen);
        if (recvLen <= 0) {
            std::cerr << "No data received or timeout." << std::endl;
            return false; // 데이터 수신 실패
        }

        std::string receivedData(packet, recvLen);

        // 헤더 패킷 처리: 새로운 이미지의 시작
        if (receivedData.find("WINDOW_NAME=") == 0) {
            size_t endPos = receivedData.find('\n');
            if (endPos != std::string::npos) {
                currentWindowName = receivedData.substr(12, endPos - 12); // 윈도우 이름 추출
                buffer.clear();  // 이전 이미지 데이터 초기화
                collectingData = true; // 새로운 이미지 수집 활성화
                continue; // 다음 패킷으로 넘어감
            }
        }

        // 이미지 데이터 처리
        if (collectingData) {
            buffer.insert(buffer.end(), packet, packet + recvLen); // 데이터 추가

            // JPEG 데이터의 끝(0xFFD9) 확인 및 이미지 처리
            if ((recvLen < DF_UDP_BUFFER_SIZE) || 
                (recvLen >= 2 && (unsigned char)packet[recvLen - 2] == 0xFF && (unsigned char)packet[recvLen - 1] == 0xD9)) {
                
                // OpenCV 이미지 디코딩
                image = cv::imdecode(buffer, cv::IMREAD_COLOR);
                if (image.empty()) {
                    std::cerr << "Failed to decode image for window: " << currentWindowName << std::endl;
                    buffer.clear();
                    collectingData = false;
                    return false;
                }

                // 결과 반환
                windowName = currentWindowName;
                buffer.clear();
                collectingData = false; // 데이터 수집 완료
                break; // 루프 종료
            }
        }
    }

    return true; // 성공적으로 이미지 수신
}



void UDPReceiver::processImage(const std::vector<uchar>& buffer, const std::string& windowName) {
    Mat image = imdecode(buffer, IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to decode image." << std::endl;
        return;
    }

    // 윈도우 이름에 따라 출력
    imshow(windowName, image);
    waitKey(1);
}




Flight_STATE UDPReceiver::getFlightState()
{
  std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
  return m_flight_state;
}
SW_STATE UDPReceiver::getSwitchState()
{
  std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
  return m_sw_state;
}