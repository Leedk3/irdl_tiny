#ifndef UNI_H
#define UNI_H

#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

class Uni {
public:
    std::string president_name;
    int established_year;
    int id;

    Uni(std::string name, int year, int university_id, int password, int reset_number);
    
    // 비밀번호가 맞는지 확인하고 정보를 출력하는 새로운 함수
    void check_and_print(int input_pw);

private:
    int pw;
    int rn;
    void print_info(); // 이제 정보 출력은 내부에서만 하도록 private으로 옮겨도 좋습니다.
};

#endif