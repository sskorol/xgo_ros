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
#include <vector>
#include <string>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <json/json.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  uint8_t addr;
  uint8_t length;
  uint8_t message[40];
} OrderPacket;

using namespace std;
using namespace serial;
using namespace tf2;
using namespace geometry_msgs::msg;
using namespace std_msgs::msg;
using namespace nav_msgs::msg;
using namespace rclcpp;

class XGO : public Node {
public:
  XGO();
  void readState();

private:
  unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Transform ts;
  tf2::Quaternion q;
  Odometry odom;
  rclcpp::Time current_time, last_time;

  bool enable_joint_gui;
  string com_port_str;
  char com_port[30];
  uint8_t rxFlag = 0;
  uint8_t rxBuffer[256];
  uint8_t rxData[256];
  uint8_t rxPtr = 0;

  Serial ser;
  Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_body_pose_;
  Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_arm_pose_;
  Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_joint_angle_;
  Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_leg_pose_;
  Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_order_;
  Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  float vx = 0;
  float vy = 0;
  float vyaw = 0;
  float battery;
  float body_pose[6] = {0, 0, 108, 0, 0, 0};
  float joint_angle[15];
  float leg_pose[12] = {0, 0, 108, 0, 0, 108, 0, 0, 108, 0, 0, 108};
  float arm_pose[3] = {85, 5, 55};
  float imu_angle[3];
  float imu_acc[3];
  float vx_max;
  float vy_max;
  float vyaw_max;
  vector<float> period_limit;
  vector<vector<int>> joint_limit;
  vector<vector<int>> body_limit;
  vector<vector<int>> arm_limit;

  void initParameters();
  void sendSpeed();
  void sendOrder(uint8_t addr, uint8_t data);
  void sendOrder(uint8_t addr, uint8_t data_len, uint8_t *data);
  void action(uint8_t action_id);
  void reset();
  void sendMotorAngle();
  void updateState();
  void pubJointState();
  void pubImuState();
  void pubBaseTf();
  void pubBattery();
  void publishOdometry();
  void sendBodyPose();
  void sendLegPose();
  void sendArmPose();
  bool initParam();
  bool initCOM();
  void initSubscriber();
  void initPublisher();
  void parseStringParameter(const string &param_name, vector<vector<int>> &output);
  void parsePeriodLimitString(const string &param_name, vector<float> &output);
  void sendOrder(OrderPacket packet);
  void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void legposeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void bodyposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void jointangleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void armposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void orderCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
};

float limit(float data, float min_limit, float max_limit);
uint8_t float2uint8(float data, float min_limit, float max_limit);
float uint82float(uint8_t data, float min_limit, float max_limit);
float toRad(float value);

#endif
