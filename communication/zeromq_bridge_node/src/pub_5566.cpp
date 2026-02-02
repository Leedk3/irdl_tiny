/*
$ sudo apt-get update
$ sudo apt-get install libzmq3-dev libczmq-dev

V2V(encoder:sub): 192.168.1.1(5566)
AGX(agx:pub)    : 192.168.1.2(5566)
*/

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <zmq.hpp>

#include "sdu.hpp"

int main(int argc, char *argv[])
{
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind("tcp://*:5566");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (1)
    {
        V2VB_PL0_MSG_T pl0;

        pl0.payload_type_code    = 0;
        pl0.address_qualifier    = 0;
        pl0.icao                 = 0x1ABBA1;
        pl0.security_mode        = 0;
        pl0.latitude_wgs84       = -80.12345678; 
        pl0.longitude_wgs84      = 127.87654321;
        pl0.true_altitude_ft     = 200.1;
        pl0.pressure_altitude_ft = 200.2;
        pl0.absolute_altitude_ft = 200.3;

        std::cout << "sending: " << std::hex << sizeof(V2VB_PL0_MSG_T) << std::endl;

        std::string topic = "0";
        zmq::message_t topic_msg(topic.c_str(), topic.size());

        zmq::message_t request(sizeof(V2VB_PL0_MSG_T));
        memcpy(request.data(), &pl0, sizeof(V2VB_PL0_MSG_T));
        zmq::message_t body_msg(request.data(), sizeof(V2VB_PL0_MSG_T));

        //publisher.send(topic_msg, zmq::send_flags::sndmore);
        publisher.send(topic_msg, ZMQ_SNDMORE);
        //publisher.send(body_msg, zmq::send_flags::none);
        publisher.send(body_msg, 0);

        printf("[PUB] send:\n");
        printf("  payload_type_code    = %d\n", pl0.payload_type_code);
        printf("  address_qualifier    = %d\n", pl0.address_qualifier);
        printf("  icao                 = 0x%06X\n", pl0.icao);
        printf("  security_mode        = %d\n", pl0.security_mode);
        printf("  latitude_wgs84       = %.8f\n", pl0.latitude_wgs84);
        printf("  longitude_wgs84      = %.8f\n", pl0.longitude_wgs84);
        printf("  true_altitude_ft     = %.2f\n", pl0.true_altitude_ft);
        printf("  pressure_altitude_ft = %.2f\n", pl0.pressure_altitude_ft);
        printf("  absolute_altitude_ft = %.2f\n", pl0.absolute_altitude_ft);
        printf("  ...\n");
        printf("--------------------------------------------------\n");


        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
