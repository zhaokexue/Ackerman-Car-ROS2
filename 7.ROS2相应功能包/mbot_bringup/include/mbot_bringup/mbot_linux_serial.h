#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H


#include <iostream>
#include <string>

#include <boost/asio.hpp>

using namespace std;
using namespace boost::asio;

class MbotSerial {
    public:
        MbotSerial();
        ~MbotSerial();
    
        void mbotSerialInit(std::string input_str);
        void mbotWrite(const short leftSpeed, const short rightSpeed,const short frontAngle, const unsigned char ctrlFlag);
        bool mbotRead(short &leftSpeed, short &rightSpeed,short &Angle, unsigned char &ctrlFlag);
        std::string string2hex(const std::string& input);
        unsigned char getCrc8(unsigned char *ptr, unsigned short len);
        void setSerialName(std::string input_str);

    private:
        //发送左右轮速控制速度共用体,舵机控制角度
        union sendData
        {
            short d;
            unsigned char data[2];
        }leftVelSet,rightVelSet,frontAngSet;
        //接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
        union receiveData
        {
            short d;
            unsigned char data[2];
        }leftVelNow,rightVelNow,angleNow;
    
        boost::system::error_code err;
        boost::asio::io_service iosev;
        boost::asio::serial_port sp;
        std::string serialName;
};

#endif