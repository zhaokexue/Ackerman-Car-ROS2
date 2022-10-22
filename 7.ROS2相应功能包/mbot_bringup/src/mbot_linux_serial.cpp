#include "mbot_linux_serial.h"

using namespace std;
using namespace boost::asio;

/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

// 构造函数
MbotSerial::MbotSerial() : 
    sp(iosev)
{}

// 析构函数
MbotSerial::~MbotSerial(){}

// 串口参数初始化
void MbotSerial::mbotSerialInit(std::string input_str)
{
    setSerialName(input_str);

    sp.open(serialName, err);
    
    if(err){
        std::cout << "Error: " << err << std::endl;
        std::cout << "请检查您的串口名称，是否已经准备好：\n 1.读写权限是否打开（默认不打开) \n 2.串口名称是否正确" << std::endl;
        return ;
    }    

    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8)); 
}

// 通过串口发送相关数据到STM32
void MbotSerial::mbotWrite(const short leftSpeed, const short rightSpeed,const short frontAngle, const unsigned char ctrlFlag) 
{
    unsigned char buf[12] = {0};
    int i, length = 0;

    // * 左右电机
    leftVelSet.d  = leftSpeed; 
    rightVelSet.d = rightSpeed;
    // * 舵机角度 
    frontAngSet.d = frontAngle;

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];               //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 7;
    buf[2] = length;                      //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data[i]; //buf[5] buf[6]
        buf[i + 7] = frontAngSet.data[i]; //buf[7] buf[8]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[9]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[10]
    buf[3 + length + 1] = ender[0];      //buf[11]
    buf[3 + length + 2] = ender[1];      //buf[12]

    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}

// 通过串口从STM32读数据
bool MbotSerial::mbotRead(short &leftSpeed, short &rightSpeed,short &Angle, unsigned char &ctrlFlag)
{
    // 0  1  2  3  4  5  6  7  8  9  10 11 12 
    // 55 AA 07 00 00 00 00 01 00 00 0F 0D 0A
    char length = 0;
    unsigned char checkSum;
    unsigned char buf[15]={0};
    bool succeedReadFlag = false;
    //=========================================================
    // 此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n", err);   // 第一次分割数据 根据数据尾"\r\n"

        std::string str;
        std::istream is(&response);
        // is >> str; // 这个会丢 \r\n
        
        while(response.size() != 0)
        {
            std::getline(is, str, (char)header[0]);          // 第二次分割数据 根据数据头 这个会丢 0x55
            // std::cout <<"筛选前："<<" {"<< string2hex(str) << "} " <<std::endl;
            if(str.size() == 12) 
            {
                std::string finalStr(1, header[0]);
                finalStr = finalStr + str;

                // std::cout <<"筛选后："<<" {"<< string2hex(finalStr) << "} " <<std::endl;   
                for (size_t i = 0; i < finalStr.size(); i++)
                {
                    buf[i] = finalStr[i];
                }
                succeedReadFlag = true;
                break;
            }
            else
            {
                continue;
            }
        }
    }  
    catch(boost::system::system_error &err)
    {
        std::cout << "read_until error" << std::endl;
    } 
    //=========================================================       

    if(succeedReadFlag)
    {
        // 检查信息头
        if (buf[0] != header[0] || buf[1] != header[1])   //buf[0] buf[1]
        {
            std::cout << "Received message header error!" << std::endl;
            return false;
        }
        // 数据长度
        length = buf[2];                                 //buf[2]

        // 检查信息校验值
        checkSum = getCrc8(buf, 3 + length);             //buf[10] 计算得出
        if (checkSum != buf[3 + length])                 //buf[10] 串口接收
        {
            std::cout << "Received data check sum error!" << std::endl;
            return false;
        }    

        // 读取速度值
        for(int i = 0; i < 2; i++)
        {
            leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
            rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
            angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
        }

        leftSpeed = leftVelNow.d;
        rightSpeed = rightVelNow.d;
        Angle = angleNow.d;
        ctrlFlag = buf[9];

        return true;
    }
    else
    {
        return false;
    }

}

// 将字符串转化为为16进制的字符串输出,调试显示用
std::string MbotSerial::string2hex(const std::string& input)
{
    static const char* const lut = "0123456789ABCDEF";
    size_t len = input.length();

    std::string output;
    output.reserve(2 * len);
    for (size_t i = 0; i < len; ++i)
    {
        const unsigned char c = input[i];
        output.push_back(lut[c >> 4]);
        output.push_back(lut[c & 15]);
    }
    return output;
}

// 获得8位循环冗余校验值 , 校验用
unsigned char MbotSerial::getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}

// 根据需求自定义串口通信的端口名称
void MbotSerial::setSerialName(std::string input_str)
{
    serialName = input_str;
}