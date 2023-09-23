#ifndef __XGO_HPP
#define __XGO_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <json/json.h>
#include <vector>
#include <string>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <json/json.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

using namespace std;
using namespace serial;
using namespace tf2;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;

typedef struct
{
  uint8_t addr;
  uint8_t length;
  uint8_t message[40];
} OrderPacket;

struct Config {
  string comPort;
  float vxMax;
  float vyMax; 
  float vYawMax;
  vector<float> periodLimit;
  vector<vector<int>> jointLimit;
  vector<vector<int>> bodyLimit;
  vector<vector<int>> armLimit;
};

class XGO : public Node
{
public:
  XGO();
  void readState();

private:
  Subscription<geometry_msgs::msg::Twist>::SharedPtr velocitySubscriber;
  Subscription<geometry_msgs::msg::Pose>::SharedPtr bodyPoseSubscriber;
  Subscription<geometry_msgs::msg::Pose>::SharedPtr armPoseSubscriber;
  Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher;
  Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
  Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batteryPublisher;
  Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

  unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time current_time, last_time;
  rclcpp::Clock::SharedPtr clock_;
  Odometry odom;
  Serial serialInterface;

  Config config_;

  uint8_t rxFlag = 0;
  uint8_t rxBuffer[256];
  uint8_t rxData[256];
  uint8_t rxPtr = 0;

  string config_path;
  float vx = 0;
  float vy = 0;
  float vyaw = 0;
  float battery;
  float bodyPose[6] = { 0, 0, 108, 0, 0, 0 };
  float legPose[12] = { 0, 0, 108, 0, 0, 108, 0, 0, 108, 0, 0, 108 };
  float armPose[3] = { 85, 5, 55 };
  float jointAngle[15];
  float imuAngle[3];
  float imuAcceleration[3];

  Config loadConfig();
  void sendSpeed();
  void sendOrder(uint8_t addr, uint8_t data);
  void sendOrder(uint8_t addr, uint8_t data_len, uint8_t* data);
  void sendOrder(OrderPacket packet);
  void action(uint8_t action_id);
  void reset();
  void updateState();
  void publishJointState();
  void publishImuState();
  void publishBaseTransformations();
  void publishBattery();
  void publishOdometry();
  void sendBodyPose();
  void sendArmPose();
  bool initCOM();
  void initSubscribers();
  void initPublishers();
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void armPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
};

#endif
