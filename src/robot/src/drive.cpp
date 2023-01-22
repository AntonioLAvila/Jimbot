#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string.h>
#include <bits/stdc++.h>

#include "glm/vec2.hpp"
#include "glm/glm.hpp"

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber") {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(//sensor_msgs::msg::Imu
      "/joy"/*"/camera/gyro/sample"*/, rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

    void updateWheelVectors(glm::vec2 trans, float rot){
      wheelPowers[0] = glm::dot(trans, wheelUnitVectors[0]) + rot;
      wheelPowers[1] = glm::dot(trans, wheelUnitVectors[1]) - rot;
      wheelPowers[2] = glm::dot(trans, wheelUnitVectors[2]) - rot;
      wheelPowers[3] = glm::dot(trans, wheelUnitVectors[3]) + rot;
    }

    void setMotors(){
    }

  private:
    float rotate = 0;
    glm::vec2 translate = glm::vec2(0,0);

    glm::vec2 wheelUnitVectors[4] = {glm::vec2(1/sqrt(2), 1/sqrt(2)),
                                    glm::vec2(-1/sqrt(2), 1/sqrt(2)),
                                    glm::vec2(1/sqrt(2), 1/sqrt(2)),
                                    glm::vec2(-1/sqrt(2), 1/sqrt(2))};
    
    glm::vec2 wheelVectors[4] = {glm::vec2(0,0),
                                glm::vec2(0,0),
                                glm::vec2(0,0),
                                glm::vec2(0,0)};

    float wheelPowers[4] = {0,0,0,0};

    void topic_callback(const sensor_msgs::msg::Joy & msg) {
      // geometry_msgs::msg::Vector3 v = msg.angular_velocity;
      
      // RCLCPP_INFO(this->get_logger(), to_string(v.x).c_str());

      translate.x = msg.axes[0];
      translate.y = msg.axes[1];
      rotate = msg.axes[3];
      RCLCPP_INFO(this->get_logger(), std::to_string(msg.axes[0]).c_str());
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}