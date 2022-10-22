#include "mbot.h" 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);									
    rclcpp::spin(std::make_shared<Mbot>());
    rclcpp::shutdown();
    return 0;
}


