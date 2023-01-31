#include <memory>

#include <stdio.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string.h>
#include <bits/stdc++.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber") {/*sensor_msgs::msg::Imu*/
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy"/*"/camera/gyro/sample"*/, rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

      serial_port = open("/dev/ttyUSB0", O_RDWR);

      // Create new termios struct, we call it 'tty' for convention
      // struct termios tty;

      // Read in existing settings, and handle any error
      if(tcgetattr(serial_port, &tty) != 0) {
          printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      }

      tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
      tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
      tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
      tty.c_cflag |= CS8; // 8 bits per byte (most common)
      tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
      tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      tty.c_lflag &= ~ICANON;
      tty.c_lflag &= ~ECHO; // Disable echo
      tty.c_lflag &= ~ECHOE; // Disable erasure
      tty.c_lflag &= ~ECHONL; // Disable new-line echo
      tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
      tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
      tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

      tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
      // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
      // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

      tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
      tty.c_cc[VMIN] = 0;

      // Set in/out baud rate to be 9600
      cfsetispeed(&tty, B9600);
      cfsetospeed(&tty, B9600);

      // Save tty settings, also checking for error
      if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
          printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      }
    }

    void updateWheelPowers(float trans[2], float rot){
      wheelPowers[0] = (trans[0] + trans[1] + rot)/2;
      wheelPowers[1] = (trans[0] - trans[1] - rot)/2;
      wheelPowers[2] = (trans[0] + trans[1] - rot)/2;
      wheelPowers[3] = (trans[0] - trans[1] + rot)/2;
    }

    void setMotors(float powers[4]){
      char command[25];
      sprintf(command, "FL%+4.fFR%+4.fBR%+4.fBL%+4.f\n", powers[0], powers[1], powers[2], powers[3]);
      write(serial_port, command, sizeof(command));
    }

  private:
    float rotate = 0;
    float translate[2] = {0,0};
    float wheelPowers[4] = {0,0,0,0};
    int serial_port;
    struct termios tty;

    void topic_callback(const sensor_msgs::msg::Joy & msg) {
      // geometry_msgs::msg::Vector3 v = msg.angular_velocity;
      
      // RCLCPP_INFO(this->get_logger(), to_string(v.x).c_str());

      translate[0] = msg.axes[0];
      translate[1] = msg.axes[1];
      rotate = msg.axes[3];
      updateWheelPowers(translate, rotate);
      setMotors(wheelPowers);
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