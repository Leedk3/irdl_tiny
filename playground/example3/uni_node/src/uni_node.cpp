#include "uni.h"

Uni::Uni(std::string name, int year, int university_id, int password, int reset_number) 
    : president_name(name), established_year(year), id(university_id), pw(password), rn(reset_number) {}

// 비밀번호 검사 및 출력 로직
void Uni::check(int input_pw) {
    if (input_pw == pw) {
        std::cout << "\n[인증 성공] 접근을 허가합니다." << std::endl;
        print_info(); // 인증 성공 시에만 정보 출력
    } else {
        std::cout << "[인증 실패] 비밀번호가 틀렸습니다!" << std::endl;
    }
}

void Uni::print_info() {
    std::cout << "========= " << president_name << " 정보 =========" << std::endl;
    std::cout << "설립연도: " << established_year << "년" << std::endl;
    std::cout << "학교 ID: " << id << std::endl;
    std::cout << "================================" << std::endl;
}

// void Uni::print_info() {
//     // 1. 문자열(%s), 정수(%d) 자리를 미리 만들어두고 뒤에서 채워줌
//     RCLCPP_INFO(rclcpp::get_logger("uni_node"), 
//         "\n================================\n"
//         "  대학교    : %s\n"
//         "  설립연도  : %d년\n"
//         "  학교 ID   : %d\n"
//         "================================", 
//         president_name.c_str(), established_year, id);
// }
// ROS2 만의 출력 방식 위에는 c++의 터미널 표기 방식

// instance 와 object 비교 > class라는 붕어빵 틀 그걸 통해서 나오는 것들을 전부 object라 부르고 실제로 나온 것은 instance라 지칭 
// class Uni에서 나온 instance는 jbnu, kaist 만약에 다른 class에서 나온 instance 

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    Uni jbnu("전북대", 1947, 101, 1234, 777);
    Uni kaist("카이스트", 1971, 202, 5678, 888);

    std::string school_name;
    int input_pw;

    while (rclcpp::ok()) {
        std::cout << "\n조회할 학교 입력 (jbnu / kaist / quit): ";
        std::cin >> school_name;

        if (school_name == "quit") break;

        std::cout << "비밀번호를 입력하세요: ";
        std::cin >> input_pw;

        if (school_name == "jbnu") {
            jbnu.check_and_print(input_pw);
        } else if (school_name == "kaist") {
            kaist.check_and_print(input_pw);
        } else {
            std::cout << "등록되지 않은 학교입니다." << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}