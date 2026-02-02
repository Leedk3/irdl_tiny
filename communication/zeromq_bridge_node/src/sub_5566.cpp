/*
$ sudo apt-get update
$ sudo apt-get install libzmq3-dev libczmq-dev

V2V(agx:pub_5566) : 192.168.1.1(5566)
AGX(agx:sub_5566) : 192.168.1.2(5566)
*/

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdint>
#include <iomanip>  // fixed, setprecision
#include <zmq.hpp>

#include "sdu.hpp"

int main(int argc, char *argv[])
{
    zmq::context_t context(2);	// 2개의  I/O 쓰레드 처리
    zmq::socket_t subscriber(context, zmq::socket_type::sub);

    subscriber.connect("tcp://localhost:5566");
    //subscriber.connect("tcp://192.168.1.1:5566");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    uint32_t i = 0;

    while (1)
    {
        zmq::message_t topic;
        zmq::message_t msg;
        msg.rebuild();

        bool got_topic = subscriber.recv(&topic, ZMQ_DONTWAIT);

        if (got_topic) {
            std::string topic_msg(static_cast<char*>(topic.data()), topic.size());
            printf("[SUB] Received:\n");
            printf("  topic    = %s\n", topic_msg.c_str());

            bool got_msg = subscriber.recv(&msg, ZMQ_DONTWAIT);
      
            if (!strcmp(topic_msg.c_str(), "0") && got_msg && msg.size() > 1) {
                if (msg.size() == sizeof(V2VB_PL0_MSG_T)) {
                    V2VB_PL0_MSG_T received;
                    memcpy(&received, msg.data(), sizeof(V2VB_PL0_MSG_T));
      
                    printf("[SUB] Received:\n");
                    printf("  payload_type_code    = %d\n", received.payload_type_code);
                    printf("  address_qualifier    = %d\n", received.address_qualifier);
                    printf("  icao                 = 0x%06X\n", received.icao);
                    printf("  security_mode        = %d\n", received.security_mode);
                    printf("  latitude_wgs84       = %.8f\n", received.latitude_wgs84);
                    printf("  longitude_wgs84      = %.8f\n", received.longitude_wgs84);
                    printf("  true_altitude_ft     = %.2f\n", received.true_altitude_ft);
                    printf("  pressure_altitude_ft = %.2f\n", received.pressure_altitude_ft);
                    printf("  absolute_altitude_ft = %.2f\n", received.absolute_altitude_ft);
                    printf("  ...\n");
                    printf("--------------------------------------------------\n");
                }
            }
       
            if (!got_msg) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                //std::cout << "count " << i++ << std::endl;
                continue;
            }
        }
    }
}