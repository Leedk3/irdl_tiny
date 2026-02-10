#ifndef UNI_H
#define UNI_H

#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

class Uni {
public:
    // 공개 멤버 변수: 외부에서 접근 가능
    std::string president_name;
    int established_year;
    int id;

    // 생성자: 객체 생성 시 초기화 담당 .cpp파일에서는 정의(definition) .h에서는 선언(declaration)
    Uni(std::string name, int year, int university_id, int password, int reset_number);

    // 공개 멤버 함수: 정보를 출력하는 동작 (void)
    void print_info();

private:
    // 은닉 멤버 변수: 클래스 내부에서만 접근 가능
    int pw;
    int rn;
};

#endif