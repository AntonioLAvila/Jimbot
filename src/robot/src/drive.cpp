#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string.h>
#include <bits/stdc++.h>

using std::placeholders::_1;
using namespace std;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(//sensor_msgs::msg::Imu
      "/joy"/*"/camera/gyro/sample"*/, rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Joy & msg) const
    {
      // geometry_msgs::msg::Vector3 v = msg.angular_velocity;
      
      // RCLCPP_INFO(this->get_logger(), to_string(v.x).c_str());      
      RCLCPP_INFO(this->get_logger(), to_string(msg.axes[0]).c_str());
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}