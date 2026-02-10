#include "uni.h"

// 생성자 상세 구현: 초기화 리스트를 사용해 주머니를 채웁니다.
Uni::Uni(std::string name, int year, int university_id, int password, int reset_number) 
    : president_name(name), established_year(year), id(university_id), pw(password), rn(reset_number)
    {
        std::cout << std::string 대학교 정보 << std::endl;
    }


// 정보 출력 함수: cout과 endl을 사용해 화면에 표시합니다.
void Uni::print_info() {
    std::cout << "================================" << std::endl;
    std::cout << "대학교: " << president_name << std::endl;
    std::cout << "설립연도: " << established_year << "년" << std::endl;
    std::cout << "학교 ID: " << id << std::endl;
    std::cout << "================================" << std::endl;
}

int main(int argc, char ** argv) {
    // 1. ROS2 시스템 초기화
    rclcpp::init(argc, argv);

    // 2. Uni 클래스를 바탕으로 실제 객체(Instance) 생성
    Uni jbnu("양오봉", 1947, 101, 1234);
    Uni kaist("이광형", 1971, 202, 5678);

    // 3. 각 객체의 동작(함수) 호출
    jbnu.print_info();
    kaist.print_info();

    // 4. ROS2 종료
    rclcpp::shutdown();
    return 0;
}