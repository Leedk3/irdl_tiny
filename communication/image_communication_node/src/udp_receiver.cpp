#include <image_communication/udp_receiver.h>
#include <chrono>
#include <iomanip> // setprecision을 사용하기 위해 필요

using namespace std::chrono_literals;
using namespace cv;
#define INFINITE 999999

ImageReceiver::ImageReceiver() 
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

ImageReceiver::~ImageReceiver() {
    close(Socket);
    close(imageSocket);

}

// void ImageReceiver::start() {
//     std::thread flightThread(&ImageReceiver::receiveData, this);
//     std::thread imageThread(&ImageReceiver::receiveImageData, this);

//     flightThread.join();
//     imageThread.join();
// }



bool ImageReceiver::receiveUdpImageOnce(cv::Mat &image)
{
    socklen_t addrLen = sizeof(imageAddr);

    // 버퍼 준비
    char packet[DF_UDP_BUFFER_SIZE];

    // UDP 데이터 수신 (단일 패킷)
    int recvLen = recvfrom(imageSocket, packet, DF_UDP_BUFFER_SIZE, 0,
                           (struct sockaddr*)&imageAddr, &addrLen);
    if (recvLen <= 0) {
        std::cerr << "No data received or recv error." << std::endl;
        return false;
    }

    // 수신한 바이너리를 OpenCV용 vector<uchar>로 변환
    std::vector<uchar> buffer(packet, packet + recvLen);
    std::cout <<  recvLen << std::endl;

    // // JPEG -> Mat 디코딩
    // auto in_image = cv2.cvtColor(packet, cv2.COLOR_BGR2YUV_I420);

    // // image = cv::imdecode(buffer, cv::IMREAD_COLOR);
    // if (image.empty()) {
    //     std::cerr << "Failed to decode image" << std::endl;
    //     return false;
    // }

    // 디코딩 성공
    return true;
}

// bool ImageReceiver::receiveImageData(cv::Mat &image, std::string &windowName) {
//     socklen_t addrLen = sizeof(imageAddr);
//     char packet[DF_UDP_BUFFER_SIZE];
//     std::vector<uchar> buffer;                  // 이미지 데이터를 저장할 버퍼
//     std::string currentWindowName;              // 현재 수신 중인 윈도우 이름
//     bool collectingData = false;                // 데이터 수집 중인지 확인

//         int recvLen = recvfrom(imageSocket, packet, DF_UDP_BUFFER_SIZE, 0, (struct sockaddr*)&imageAddr, &addrLen);
//         if (recvLen <= 0) {
//             std::cerr << "No data received or timeout." << std::endl;
//             return false; // 데이터 수신 실패
//         }

//         std::string receivedData(packet, recvLen);

//         // 헤더 패킷 처리: 새로운 이미지의 시작
//         // if (receivedData.find("WINDOW_NAME=") == 0) {
//         //     size_t endPos = receivedData.find('\n');
//         //     if (endPos != std::string::npos) {
//         //         currentWindowName = receivedData.substr(12, endPos - 12); // 윈도우 이름 추출
//         //         buffer.clear();  // 이전 이미지 데이터 초기화
//         //         collectingData = true; // 새로운 이미지 수집 활성화
//         //         continue; // 다음 패킷으로 넘어감
//         //     }
//         // }

//         // 이미지 데이터 처리
//         if (collectingData) {
//             buffer.insert(buffer.end(), packet, packet + recvLen); // 데이터 추가

//             // JPEG 데이터의 끝(0xFFD9) 확인 및 이미지 처리
//             if ((recvLen < DF_UDP_BUFFER_SIZE) || 
//                 (recvLen >= 2 && (unsigned char)packet[recvLen - 2] == 0xFF && (unsigned char)packet[recvLen - 1] == 0xD9)) {
                
//                 // OpenCV 이미지 디코딩
//                 image = cv::imdecode(buffer, cv::IMREAD_COLOR);
//                 if (image.empty()) {
//                     std::cerr << "Failed to decode image for window: " << currentWindowName << std::endl;
//                     buffer.clear();
//                     collectingData = false;
//                     return false;
//                 }

//                 // 결과 반환
//                 windowName = currentWindowName;
//                 buffer.clear();
//                 collectingData = false; // 데이터 수집 완료
//                 break; // 루프 종료
//             }
//         }


//     // while (true) {  // 하나의 완전한 이미지를 수신할 때까지 반복
//     //     int recvLen = recvfrom(imageSocket, packet, DF_UDP_BUFFER_SIZE, 0, (struct sockaddr*)&imageAddr, &addrLen);
//     //     if (recvLen <= 0) {
//     //         std::cerr << "No data received or timeout." << std::endl;
//     //         return false; // 데이터 수신 실패
//     //     }

//     //     std::string receivedData(packet, recvLen);

//     //     // 헤더 패킷 처리: 새로운 이미지의 시작
//     //     if (receivedData.find("WINDOW_NAME=") == 0) {
//     //         size_t endPos = receivedData.find('\n');
//     //         if (endPos != std::string::npos) {
//     //             currentWindowName = receivedData.substr(12, endPos - 12); // 윈도우 이름 추출
//     //             buffer.clear();  // 이전 이미지 데이터 초기화
//     //             collectingData = true; // 새로운 이미지 수집 활성화
//     //             continue; // 다음 패킷으로 넘어감
//     //         }
//     //     }

//     //     // 이미지 데이터 처리
//     //     if (collectingData) {
//     //         buffer.insert(buffer.end(), packet, packet + recvLen); // 데이터 추가

//     //         // JPEG 데이터의 끝(0xFFD9) 확인 및 이미지 처리
//     //         if ((recvLen < DF_UDP_BUFFER_SIZE) || 
//     //             (recvLen >= 2 && (unsigned char)packet[recvLen - 2] == 0xFF && (unsigned char)packet[recvLen - 1] == 0xD9)) {
                
//     //             // OpenCV 이미지 디코딩
//     //             image = cv::imdecode(buffer, cv::IMREAD_COLOR);
//     //             if (image.empty()) {
//     //                 std::cerr << "Failed to decode image for window: " << currentWindowName << std::endl;
//     //                 buffer.clear();
//     //                 collectingData = false;
//     //                 return false;
//     //             }

//     //             // 결과 반환
//     //             windowName = currentWindowName;
//     //             buffer.clear();
//     //             collectingData = false; // 데이터 수집 완료
//     //             break; // 루프 종료
//     //         }
//     //     }
//     // }

//     return true; // 성공적으로 이미지 수신
// }



void ImageReceiver::processImage(const std::vector<uchar>& buffer, const std::string& windowName) {
    Mat image = imdecode(buffer, IMREAD_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to decode image." << std::endl;
        return;
    }

    // 윈도우 이름에 따라 출력
    imshow(windowName, image);
    waitKey(1);
}




// Flight_STATE ImageReceiver::getFlightState()
// {
//   std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
//   return m_flight_state;
// }
// SW_STATE ImageReceiver::getSwitchState()
// {
//   std::lock_guard<std::mutex> lock(state_mutex); // Lock while accessing shared state
//   return m_sw_state;
// }