#include "udp_communication/udp_sender.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <sstream>  

UDPSender::UDPSender()
  : socket_fd_(-1)
  , is_initialized_(false)
{
}

UDPSender::~UDPSender()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool UDPSender::initSocket(const std::string &ip, int port)
{
  // 소켓 생성
  socket_fd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ == -1) {
    std::cerr << "[UDPSender] Failed to create socket()" << std::endl;
    return false;
  }

  // 재사용 옵션 등 설정(필요 시)
  int enable = 1;
  setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

  // 서버 주소 설정
  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family      = PF_INET;
  server_addr_.sin_port        = htons(port);
  server_addr_.sin_addr.s_addr = inet_addr(ip.c_str());

  if (server_addr_.sin_addr.s_addr == INADDR_NONE) {
    std::cerr << "[UDPSender] Invalid IP address: " << ip << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  is_initialized_ = true;
  std::cout << "[UDPSender] Socket initialized: " << ip << ":" << port << std::endl;
  return true;
}

// int UDPSender::sendData(const AVI_STATE &data)
// {
//   if (!is_initialized_ || socket_fd_ < 0) {
//     std::cerr << "[UDPSender] Socket not initialized.\n";
//     return -1;
//   }

//   // 예: 사용자 구조체 전체를 바이너리로 전송하는 방식
//   //     또는 문자열(예: " AILERON=...")을 만들어 전송하는 방식 중 택 1
//   // 여기서는 간단히 구조체 전체를 sendto()로 보내는 예시
//   ssize_t sent_bytes = sendto(
//     socket_fd_,
//     reinterpret_cast<const char *>(&data), // 구조체 포인터
//     sizeof(data),
//     0,
//     (struct sockaddr *)&server_addr_,
//     sizeof(server_addr_)
//   );

//   if (sent_bytes < 0) {
//     std::cerr << "[UDPSender] Failed to send data.\n";
//   }
//   return static_cast<int>(sent_bytes);
// }

int UDPSender::sendData(const AVI_STATE &data)
{
  if (!is_initialized_ || socket_fd_ < 0) {
    std::cerr << "[UDPSender] Socket not initialized.\n";
    return -1;
  }

  // 1) 문자열 포맷 만들기
  //    RecvUDPAviation() 함수가 " AILERON=... \n ELEVATOR=... \n" 형태를 찾고 있으니
  //    똑같은 키워드와 구분자를 써야 합니다.
  std::stringstream ss;
  ss << " AILERON="  << data.avi_aileron << "\n"
     << " ELEVATOR=" << data.avi_elevator << "\n"
     << " RUDDER="   << data.avi_rudder   << "\n"
     << " THROTTLE=" << data.avi_throttle1 << "\n\n";
  // 만약 throttle1~4를 모두 다르게 보낼 계획이라면
  //   " THROTTLE1=...", " THROTTLE2=..." 등으로 나누셔야 하고,
  // 현재 RecvUDPAviation() 코드는 THROTTLE=... 하나만 파싱하므로
  // 기존 로직과 같게 하려면 throttle1 값만 넣어주면 됩니다.

  std::string send_str = ss.str();

  // 2) sendto()로 문자열 전송
  ssize_t sent_bytes = sendto(
    socket_fd_,
    send_str.c_str(),
    send_str.size(),
    0,
    (struct sockaddr *)&server_addr_,
    sizeof(server_addr_)
  );

  if (sent_bytes < 0) {
    std::cerr << "[UDPSender] Failed to send data.\n";
  } else {
    std::cout << "[UDPSender] Sent " << sent_bytes << " bytes: " << send_str << std::endl;
  }
  return static_cast<int>(sent_bytes);
}