#include <vector>
#include "mbot.h"

using namespace std;
std::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3,1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};
std::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3,1e-9, 0, 0, 0, 
    0, 0, 1e6, 0, 0, 0, 
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};

#define PI           (3.1415926)
#define ROBOT_RADIUS (0.032)   //m
#define ROBOT_TRACK  (0.062)   //m
#define CAR_LENGTH   (0.243)   //m
double RobotV_  = 0;
double YawRate_ = 0;

// 速度控制消息的回调函数
void cmdCallback(const geometry_msgs::msg::Twist& msg)
{
    RobotV_  = msg.linear.x;  //m/s
    YawRate_ = msg.angular.z; //rad/s
}

Mbot::Mbot():
    Node("mbot_bringup"),
    x_(0.0), y_(0.0), th_(0.0),
    vx_(0.0), vy_(0.0), vth_(0.0),
    sendLeftSpeed_(0),sendRightSpeed_(0),sendFrontAngle_(0),sendFlag_(0),
    readLeftSpeed_(0),readRightSpeed_(0),readYaw_(0),readFlag_(0)
{
    // 实例化参数列表
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_frame_id", "base_link");
    this->declare_parameter<std::string>("sub_twist", "cmd_vel");
    this->declare_parameter<bool>("diffCar_or_ackerCar", false);
    // 从系统中获取参数值
    this->get_parameter("serial_port", serial_name_);
    this->get_parameter("odom_topic", odom_topic_name_);
    this->get_parameter("odom_frame_id", odom_frame_name_);
    this->get_parameter("base_frame_id", base_frame_name_);
    this->get_parameter("sub_twist", sub_twist_name_);
    this->get_parameter("diffCar_or_ackerCar", car_type_select_);

    // 串口初始化连接
    mbotSerial_.mbotSerialInit(serial_name_);
    // 时间相关初始化
    current_time_ = this->get_clock()->now();
    last_time_ = this->get_clock()->now();	
    // 实例化定时器,50Hz
    timer_ = this->create_wall_timer(20ms, std::bind(&Mbot::mainThread,this));
    // 订阅cmd_vel
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(sub_twist_name_,10, cmdCallback);
    // 发布odom
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_,50);
    // 发布tf
    odom_t_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

Mbot::~Mbot(){}

void Mbot::diffCar(const double RobotV,const double YawRate)
{
    double r = RobotV / YawRate; // m
    // std::cout << "RobotV = " << RobotV << "YawRate = " << YawRate << std::endl;
    if(RobotV == 0)       // 旋转
    {
        sendLeftSpeed_  = (short)(-YawRate * 1000.0 * ROBOT_RADIUS);//mm/s
        sendRightSpeed_ = (short)(YawRate * 1000.0 * ROBOT_RADIUS);//mm/s
    }
    else if(YawRate == 0) // 直线
    {
        sendLeftSpeed_  = (short)(RobotV * 1000.0);//mm/s
        sendRightSpeed_ = (short)(RobotV * 1000.0);
    }
    else                  // 左右轮速度不一致
    {
        sendLeftSpeed_  = (short)(YawRate * 1000.0 * (r - ROBOT_RADIUS));//mm/s
        sendRightSpeed_ = (short)(YawRate * 1000.0 * (r + ROBOT_RADIUS));
    }
    // std::cout << "sendLeftSpeed_ = " << sendLeftSpeed_ << " sendRightSpeed_ = " << sendRightSpeed_ << std::endl;
}

void Mbot::ackerCar(const double RobotV,const double YawRate)
{
    double r = RobotV / YawRate; // m
    if(RobotV == 0)       // ackermann car can't trun rotation
    {
        sendLeftSpeed_  = 0;
        sendRightSpeed_ = 0;
        sendFrontAngle_ = 0;
    }
    else if(YawRate == 0) // Pure forward/backward motion
    {
        sendLeftSpeed_  = (short)(RobotV * 1000.0);//mm/s
        sendRightSpeed_ = (short)(RobotV * 1000.0);
        sendFrontAngle_ = 0;
    }
    else                  // Rotation about a point in space
    {
        sendLeftSpeed_  = (short)(YawRate * 1000.0 * (r - ROBOT_RADIUS));//mm/s
        sendRightSpeed_ = (short)(YawRate * 1000.0 * (r + ROBOT_RADIUS));

        // 阿克曼约束一：后左右车轮转动需同向
        if(RobotV > 0)
        {
            if(sendLeftSpeed_ < 0) {sendLeftSpeed_ = 0;}
            if(sendRightSpeed_ < 0) {sendRightSpeed_ = 0;}
        }
        else if(RobotV < 0)
        {
            if(sendLeftSpeed_ > 0) {sendLeftSpeed_ = 0;}
            if(sendRightSpeed_ > 0) {sendRightSpeed_ = 0;}            
        }
        // 阿克曼约束二：满足前轮转向和后轮的速度关系
        // # calculate the front steer servo
        sendFrontAngle_ = atan(CAR_LENGTH * YawRate / RobotV ) * (180.0 / PI); // Deg
        RCLCPP_INFO(this->get_logger(),"sendFrontAngle_ = %d\n",sendFrontAngle_);
        // sendFrontAngle_ = (atan(2 * YawRate * CAR_LENGTH) / (2 * RobotV - 2 * ROBOT_RADIUS * YawRate)) * (180.0 / PI);
    }   
    // std::cout << "sendLeftSpeed_ = " << sendLeftSpeed_ << " sendRightSpeed_ = " << sendRightSpeed_ << std::endl;
    // std::cout << "sendFrontAngle_ = " << sendFrontAngle_ << std::endl;
}

void Mbot::calcSpeed(const short leftSpeedNow,const short rightSpeedNow,const short yaw)
{
    // x方向速度，以及角速度
    vx_  = (rightSpeedNow + leftSpeedNow) / 2.0 / 1000.0;                       //m/s
    vth_ = (rightSpeedNow - leftSpeedNow) / (2.0 * ROBOT_RADIUS*1000.0) ;       //rad/s  
    th_  = (yaw / 50.0) * M_PI / 180.0;                                         //rad
}

/********************************************************
函数功能：根据机器人线速度和角度计算机器人里程计
入口参数：无
出口参数：无
********************************************************/
void Mbot::calcOdom(const double vx, const double vy, const double vth)
{
    rclcpp::Time curr_time;
    curr_time = this->get_clock()->now();
    
    double dt = (curr_time - last_time_).seconds();                                       //间隔时间
    double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;
    double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
    double delta_th = vth * dt;
    //相隔20ms
    RCLCPP_INFO(this->get_logger(),"dt:%f\n",dt); // s

    x_ += delta_x;
    y_ += delta_y;
    // th_ += delta_th;//实时角度信息,如果这里不使用IMU，也可以通过这种方式计算得出

    last_time_ = curr_time;  

    RCLCPP_INFO(this->get_logger(),"x_:%f\n",x_);
    RCLCPP_INFO(this->get_logger(),"y_:%f\n",y_);
    RCLCPP_INFO(this->get_logger(),"th_:%f\n",th_*57.3);

}

void Mbot::sendTfAndPubOdom()
{
    current_time_ = this->get_clock()->now();
    // 发布TF
    geometry_msgs::msg::TransformStamped odom_t;
    odom_t.header.stamp = current_time_;
    odom_t.header.frame_id = odom_frame_name_;
    odom_t.child_frame_id  = base_frame_name_;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,th_);
    odom_t.transform.translation.x = x_;
    odom_t.transform.translation.y = y_;
    odom_t.transform.translation.z = 0.0;
    odom_t.transform.rotation.x = odom_quat.x();
    odom_t.transform.rotation.y = odom_quat.y();
    odom_t.transform.rotation.z = odom_quat.z();
    odom_t.transform.rotation.w = odom_quat.w();
    
    odom_t_broadcaster_->sendTransform(odom_t);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = current_time_;
    msg.header.frame_id = odom_frame_name_;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = odom_quat.x();
    msg.pose.pose.orientation.y = odom_quat.y();
    msg.pose.pose.orientation.z = odom_quat.z();
    msg.pose.pose.orientation.w = odom_quat.w();
    msg.pose.covariance = odom_pose_covariance;

    msg.child_frame_id = base_frame_name_;
    msg.twist.twist.linear.x = vx_;
    msg.twist.twist.linear.y = vy_;
    msg.twist.twist.angular.z = vth_;
    msg.twist.covariance = odom_twist_covariance;

    pub_->publish(msg);
}

/********************************************************
函数功能：mbotRun，实现整合，并且发布TF变换和Odom
入口参数：机器人线速度和角速度，调用上面三个函数
出口参数：bool
********************************************************/
void Mbot::mbotRun(double RobotV, double YawRate)
{
    // * 1. 根据车模解析控制量
    if(car_type_select_)
    {
        diffCar(RobotV, YawRate);
    }
    else
    {
        ackerCar(RobotV, YawRate);
    }

    // * 2. 与STM32串口通信，线材一定要短且优质
    mbotSerial_.mbotWrite(sendLeftSpeed_, sendRightSpeed_, sendFrontAngle_, sendFlag_);
    mbotSerial_.mbotRead(readLeftSpeed_, readRightSpeed_, readYaw_, readFlag_);

    // * 3. 更新最新的速度和角度信息
    calcSpeed(readLeftSpeed_, readRightSpeed_, readYaw_);

    // * 4. 里程计计算
    calcOdom(vx_, vy_, vth_);

    // * 5. 发布TF和Odom
    sendTfAndPubOdom();
}

/********************************************************
函数功能：主函数线程
入口参数：机器人线速度和角速度，调用上面三个函数
出口参数：无
********************************************************/
void Mbot::mainThread()
{
    mbotRun(RobotV_, YawRate_);
}