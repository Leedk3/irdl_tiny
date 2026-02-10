#ifndef IMAGE_SENDER_COMM_H
#define IMAGE_SENDER_COMM_H

#include <string>
#include <netinet/in.h>

// 예시: 사용자가 이미 정의했을 것으로 추정되는 TX_message_data
//       실제 프로젝트에 맞게 필드를 조정하세요.
typedef struct {

  double roll;
  double pitch;
  double yaw;
  double throttle;
  // 필요하다면 더 많은 필드...
} TX_message_data;

typedef struct 
{
	double avi_aileron;
	double avi_elevator;
	double avi_rudder;
	double avi_throttle1;
	double avi_throttle2;
	double avi_throttle3;
	double avi_throttle4;
} AVI_STATE;

/**
 * @brief 간단한 UDP 송신 기능 담당 클래스
 */
class UDPSender
{
public:
  /**
   * @brief 기본 생성자
   */
  UDPSender();
  
  /**
   * @brief 소멸자
   */
  ~UDPSender();

  /**
   * @brief UDP 송신에 필요한 소켓 초기화
   * @param ip   목적지 IP (예: 192.168.1.48)
   * @param port 목적지 Port (예: 50000)
   * @return 성공 시 true, 실패 시 false
   */
  bool initSocket(const std::string &ip, int port);

  /**
   * @brief 주어진 TX_message_data 구조체를 UDP로 전송
   * @param data UDP로 보낼 데이터(사용자 정의 구조체)
   * @return 전송 성공 시 전송된 바이트 수, 실패 시 -1
   */
  int sendData(const AVI_STATE &data);

private:
  int socket_fd_;               ///< UDP 소켓 파일 디스크립터
  struct sockaddr_in server_addr_;  ///< 목적지 소켓 주소
  bool is_initialized_;         ///< 소켓 초기화 여부
};

#endif // IMAGE_SENDER_COMM_H
