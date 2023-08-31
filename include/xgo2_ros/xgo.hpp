#ifndef __XGO_HPP
#define __XGO_HPP

#include <cstring>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

typedef struct {
  uint8_t addr;
  uint8_t length;
  uint8_t message[40];
} OrderPacket;

class XGO : public rclcpp::Node {
public:
  XGO();
  void readState();

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Transform ts;
  tf2::Quaternion q;

  bool enable_joint_gui;
  std::string com_port_str;
  char com_port[30];
  uint8_t rxFlag = 0;
  uint8_t rxBuffer[256];
  uint8_t rxData[256];
  uint8_t rxPtr = 0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_body_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_arm_pose_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_joint_angle_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      sub_leg_pose_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_order_;
  serial::Serial ser;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

  float vx = 0;
  float vy = 0;
  float vyaw = 0;
  float battery;
  float body_pose[6] = {0, 0, 108, 0, 0, 0};
  float joint_angle[15];
  float leg_pose[12] = {0, 0, 108, 0, 0, 108, 0, 0, 108, 0, 0, 108};
  float arm_pose[3] = {85, 85, 3};
  float imu_angle[3];
  float imu_acc[3];
  float vx_max;
  float vy_max;
  float vyaw_max;
  std::vector<float> period_limit;
  std::vector<std::vector<int>> joint_limit;
  std::vector<std::vector<int>> body_limit;
  std::vector<std::vector<int>> arm_limit;

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
  void sendBodyPose();
  void sendLegPose();
  void sendArmPose();
  bool initParam();
  bool initCOM();
  void initSubscriber();
  void initPublisher();
  void parseStringParameter(const std::string &param_name,
                            std::vector<std::vector<int>> &output);
  void parsePeriodLimitString(const std::string &param_name,
                              std::vector<float> &output);
  void sendOrder(OrderPacket packet);
  void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void legposeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void bodyposeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void
  jointangleCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void armposeCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void orderCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
};

float limit(float data, float min_limit, float max_limit);
uint8_t float2uint8(float data, float min_limit, float max_limit);
float uint82float(uint8_t data, float min_limit, float max_limit);
float toRad(float value);

#endif
