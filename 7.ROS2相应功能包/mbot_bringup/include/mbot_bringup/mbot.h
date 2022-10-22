#ifndef MBOT_H
#define MBOT_H

#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "mbot_linux_serial.h"

using namespace std::chrono_literals; // c++14 方便的表示时间 比如下文的1000ms
// 这里我们构建一个继承于Node的Mbot类
class Mbot : public rclcpp::Node
{
    public:
        Mbot();
        ~Mbot();

        void diffCar(const double RobotV,const double YawRate);
        void ackerCar(const double RobotV,const double YawRate);
        void calcSpeed(const short leftSpeedNow,const short rightSpeedNow,const short yaw); 
        void calcOdom(const double vx, const double vy, const double vth);
        void sendTfAndPubOdom();

        void mbotRun(const double RobotV,const double YawRate);
        void mainThread();  

    private:
        // * 串口
        MbotSerial mbotSerial_;

        // * odom
        double x_;
        double y_;
        double th_;

        // * speed
        double vx_;
        double vy_;
        double vth_;

        // * send
        short sendLeftSpeed_;
        short sendRightSpeed_;
        short sendFrontAngle_;
        unsigned char sendFlag_;

        // * read
        short readLeftSpeed_;
        short readRightSpeed_;
        short readYaw_;
        unsigned char readFlag_;

        // * ros2
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_t_broadcaster_;
        rclcpp::Time current_time_, last_time_;

        // * parameters
        std::string serial_name_;     // 串口通信端口, 默认/dev/ttyUSB0
        std::string odom_topic_name_; // 发布里程计的topic，默认odom
        std::string odom_frame_name_; // 发布里程计的坐标系，默认odom
        std::string base_frame_name_; // 机器人坐标系名称，默认base_link
        std::string sub_twist_name_;  // 共能包订阅Twist话题名称，默认cmd_vel
        bool car_type_select_;        // 默认false：差速(diff_car)
};

#endif /* mbot_H */
