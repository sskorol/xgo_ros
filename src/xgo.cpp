#include "xgo2_ros/xgo.hpp"
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

float limit(float data, float min_limit, float max_limit) {
  return data < min_limit ? min_limit : data > max_limit ? max_limit : data;
}

float uint82float(uint8_t data, float min_limit, float max_limit) {
  return (float)data / 255.0 * (max_limit - min_limit) + min_limit;
}

uint8_t float2uint8(float data, float min_limit, float max_limit) {
  return (uint8_t)((data - min_limit) / (max_limit - min_limit) * 255);
}

float bytes2float(uint8_t *databytes) { return *(float *)(databytes); }

float toRad(float value) { return value / 57.3; }

XGO::XGO() : Node("xgo_control_node") {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  vx_max = 0.3f;
  vy_max = 0.2f;
  vyaw_max = 1.2f;

  joint_limit = {{-73, 57}, {-66, 93}, {-31, 31},
                 {-65, 65}, {-50, 85}, {-90, 75}};

  body_limit = {{35, -35}, {18, -18}, {75, 115},
                {-20, 20}, {-15, 15}, {-11, 11}};

  arm_limit = {{5, 95}, {-85, 150}, {-100, 150}};

  period_limit = {1.5f, 8.0f};
  enable_joint_gui = false;
  com_port_str = "/dev/ttyS0";

  // initParameters();
  initCOM();
  initSubscriber();
  initPublisher();
}

void XGO::initParameters() {
  this->declare_parameter("com_port_str");
  this->declare_parameter("vx_max");
  this->declare_parameter("vy_max");
  this->declare_parameter("vyaw_max");
  this->declare_parameter("enable_joint_gui");
  this->declare_parameter<std::string>("joint_limit", "[]");
  this->declare_parameter<std::string>("arm_limit", "[]");
  this->declare_parameter<std::string>("body_limit", "[]");
  this->declare_parameter<std::string>("period_limit", "[]");

  this->get_parameter("com_port_str", com_port_str);
  this->get_parameter("vx_max", vx_max);
  this->get_parameter("vy_max", vy_max);
  this->get_parameter("vyaw_max", vyaw_max);
  this->get_parameter("enable_joint_gui", enable_joint_gui);

  parseStringParameter("joint_limit", joint_limit);
  parseStringParameter("arm_limit", arm_limit);
  parseStringParameter("body_limit", body_limit);
  parsePeriodLimitString("period_limit", period_limit);
}

bool XGO::initCOM() {
  std::cout << "Connecting to " << com_port_str << std::endl;
  std::strcpy(com_port, com_port_str.c_str());
  serial::Timeout to = serial::Timeout::simpleTimeout(100);
  ser.setPort(com_port_str);
  ser.setBaudrate(115200);
  ser.setTimeout(to);
  ser.open();
  sendOrder(0x08, 0x01);
  return true;
}

void XGO::initSubscriber() {
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&XGO::cmdvelCallback, this, std::placeholders::_1));
  sub_body_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "body_pose", 10,
      std::bind(&XGO::bodyposeCallback, this, std::placeholders::_1));
  sub_arm_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "arm_pose", 10,
      std::bind(&XGO::armposeCallback, this, std::placeholders::_1));
  sub_joint_angle_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
          "joint_angle", 10,
          std::bind(&XGO::jointangleCallback, this, std::placeholders::_1));
  sub_leg_pose_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "leg_pose", 10,
      std::bind(&XGO::legposeCallback, this, std::placeholders::_1));
  sub_order_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "order", 10, std::bind(&XGO::orderCallback, this, std::placeholders::_1));
}

void XGO::initPublisher() {
  if (!enable_joint_gui) {
    pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
  }
  // pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",
  // 10);
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  battery_pub_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
}

void XGO::readState() {
  int readLen = ser.available();

  uint8_t tempPtr = 0;
  if (readLen > 0) {
    readLen = ser.read(rxBuffer, readLen);
    while (tempPtr != readLen) {
      switch (rxFlag) {
      case 0:
        if (rxBuffer[tempPtr] == 0x55)
          rxFlag = 1;
        break;
      case 1:
        if (rxBuffer[tempPtr] == 0x00)
          rxFlag = 2;
        else
          rxFlag = 0;
        break;
      case 2:
        rxData[rxPtr++] = rxBuffer[tempPtr];
        if (rxPtr == 40) {
          rxFlag = 3;
        }
        break;
      case 3:
        if (rxBuffer[tempPtr] == 0x00)
          rxFlag = 4;
        else
          rxFlag = 0;
        break;
      case 4:
        if (rxBuffer[tempPtr] == 0xAA)
          updateState();
        rxFlag = 0;
        rxPtr = 0;
        break;
      default:
        rxFlag = 0;
        rxPtr = 0;
        break;
      }
      tempPtr++;
    }
  }
}

void XGO::updateState() {
  battery = rxData[0];
  for (int i = 0; i < 15; i++) {
    if (i < 12) {
      joint_angle[i] = toRad(uint82float(rxData[1 + i], joint_limit[i % 3][0],
                                         joint_limit[i % 3][1]));
    } else {
      joint_angle[i] = toRad(uint82float(rxData[1 + i], joint_limit[i - 9][0],
                                         joint_limit[i - 9][1]));
    }
  }
  imu_angle[0] = toRad(bytes2float(rxData + 16));
  imu_angle[1] = toRad(bytes2float(rxData + 20));
  imu_angle[2] = toRad(bytes2float(rxData + 24));
  imu_acc[0] = bytes2float(rxData + 28);
  imu_acc[1] = bytes2float(rxData + 32);
  imu_acc[2] = bytes2float(rxData + 36);
  pubJointState();
  pubBaseTf();
  pubBattery();
}

void XGO::cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  vx = limit(msg->linear.x, -vx_max, vx_max);
  vy = limit(msg->linear.y, -vy_max, vy_max);
  vyaw = limit(msg->angular.z, -vyaw_max, vyaw_max);
  sendSpeed();
}

void XGO::jointangleCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < 15; i++) {
    joint_angle[i] = msg->data[i];
    sendMotorAngle();
  }
}

void XGO::bodyposeCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < 6; i++) {
    body_pose[i] = msg->data[i];
    sendBodyPose();
  }
}

void XGO::legposeCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < 12; i++) {
    leg_pose[i] = msg->data[i];
    sendLegPose();
  }
}

void XGO::armposeCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  for (int i = 0; i < 3; i++) {
    arm_pose[i] = msg->data[i];
    sendArmPose();
  }
}

void XGO::sendOrder(uint8_t addr, uint8_t data) {
  OrderPacket packet;
  packet.addr = addr;
  packet.length = 1;
  packet.message[0] = data;
  sendOrder(packet);
}

void XGO::sendOrder(uint8_t addr, uint8_t data_len, uint8_t *data) {
  OrderPacket packet;
  packet.addr = addr;
  packet.length = data_len;
  for (int i = 0; i < data_len; i++) {
    packet.message[i] = data[i];
  }
  sendOrder(packet);
}

void XGO::sendOrder(OrderPacket packet) {
  uint8_t data[50];
  data[0] = 0x55;
  data[1] = 0x00;
  data[2] = packet.length + 8;
  data[3] = 0x01;
  data[4] = packet.addr;
  data[5 + packet.length] = data[2] + data[3] + data[4];
  for (int i = 0; i < packet.length; i++) {
    data[5 + i] = packet.message[i];
    data[5 + packet.length] += data[5 + i];
  }
  data[5 + packet.length] = ~data[5 + packet.length];
  data[6 + packet.length] = 0x00;
  data[7 + packet.length] = 0xAA;
  ser.write(data, 8 + packet.length);
}

void XGO::orderCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  sendOrder(msg->data[0], msg->data[1], (uint8_t *)msg->data.data() + 2);
}

void XGO::action(uint8_t action_id) { sendOrder(0x3E, action_id); }

void XGO::reset() { action(0xFF); }

void XGO::sendBodyPose() {
  for (int i = 0; i < 6; i++) {
    sendOrder(0x33 + i,
              float2uint8(body_pose[i], body_limit[i][0], body_limit[i][1]));
  }
}

void XGO::sendLegPose() {
  OrderPacket packet;
  packet.addr = 0x40;
  packet.length = 12;
  for (int i = 0; i < 12; i++) {
    packet.message[i] =
        float2uint8(leg_pose[i], body_limit[i % 3][0], body_limit[i % 3][1]);
  }
  sendOrder(packet);
}

void XGO::sendArmPose() {
  sendOrder(0x71, float2uint8(arm_pose[0], arm_limit[0][0], arm_limit[0][1]));
  sendOrder(0x73, float2uint8(arm_pose[1], arm_limit[1][0], arm_limit[1][1]));
  sendOrder(0x74, float2uint8(arm_pose[2], arm_limit[2][0], arm_limit[2][1]));
}

void XGO::sendMotorAngle() {
  OrderPacket packet;
  packet.addr = 0x50;
  packet.length = 15;
  for (int i = 0; i < 15; i++) {
    if (i < 12)
      packet.message[i] = float2uint8(joint_angle[i], joint_limit[i % 3][0],
                                      joint_limit[i % 3][1]);
    else
      packet.message[i] = float2uint8(joint_angle[i], joint_limit[i - 9][0],
                                      joint_limit[i - 9][1]);
  }
  sendOrder(packet);
}

void XGO::sendSpeed() {
  sendOrder(0x30, float2uint8(vx, -vx_max, vx_max));
  sendOrder(0x31, float2uint8(vy, -vy_max, vy_max));
  sendOrder(0x32, float2uint8(vyaw, -vyaw_max, vyaw_max));
}

void XGO::pubJointState() {
  if (!enable_joint_gui) {
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = rclcpp::Clock().now();
    jointState.name = {
        "13_Joint", "12_Joint", "11_Joint", "23_Joint", "22_Joint", "21_Joint",
        "33_Joint", "32_Joint", "31_Joint", "43_Joint", "42_Joint", "41_Joint",
        "53_Joint", "52_Joint", "51_Joint", "50_Joint", "500_Joint"};
    jointState.position = {
        joint_angle[2],          joint_angle[1],          joint_angle[0],
        -joint_angle[5],         joint_angle[4],          joint_angle[3],
        -joint_angle[8],         joint_angle[7],          joint_angle[6],
        joint_angle[11],         joint_angle[10],         joint_angle[9],
        -joint_angle[14],        joint_angle[13],         joint_angle[12],
        joint_angle[12] / 110.0, -joint_angle[12] / 110.0};
    pub_joint_->publish(jointState);
  }
}

void XGO::pubImuState() {
  sensor_msgs::msg::Imu imuState;
  imuState.header.stamp = rclcpp::Clock().now();
  imuState.header.frame_id = "imu_link";
  tf2::Quaternion q;
  q.setRPY(imu_angle[0], imu_angle[1], imu_angle[2]); // roll, pitch, yaw
  imuState.orientation.x = q.x();
  imuState.orientation.y = q.y();
  imuState.orientation.z = q.z();
  imuState.orientation.w = q.w();
  imuState.linear_acceleration.x = imu_acc[0];
  imuState.linear_acceleration.y = imu_acc[1];
  imuState.linear_acceleration.z = imu_acc[2];
  pub_imu_->publish(imuState);
}

void XGO::pubBaseTf() {
  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = this->get_clock()->now();
  ts.header.frame_id = "world";
  ts.child_frame_id = "base_link";
  ts.transform.translation.x = 0.0;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.12;
  tf2::Quaternion q;
  q.setRPY(imu_angle[0], imu_angle[1], imu_angle[2]);
  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();
  tf_broadcaster_->sendTransform(ts);
}

void XGO::pubBattery() {
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = battery;
  battery_pub_->publish(msg);
}

void XGO::parseStringParameter(const std::string &param_name,
                               std::vector<std::vector<int>> &output) {
  std::string param_str;
  if (this->get_parameter(param_name, param_str)) {
    Json::Value parsed;
    Json::CharReaderBuilder rbuilder;
    std::string errs;
    std::stringstream sstream(param_str);
    if (Json::parseFromStream(rbuilder, sstream, &parsed, &errs)) {
      output.clear();
      for (const auto &element : parsed) {
        std::vector<int> inner_vec;
        for (const auto &inner_element : element) {
          inner_vec.push_back(inner_element.asInt());
        }
        output.push_back(inner_vec);
      }
    }
  }
}

void XGO::parsePeriodLimitString(const std::string &param_name,
                                 std::vector<float> &output) {
  std::string param_str;
  if (this->get_parameter(param_name, param_str)) {
    Json::Value parsed;
    Json::CharReaderBuilder rbuilder;
    std::string errs;
    std::stringstream sstream(param_str);
    if (Json::parseFromStream(rbuilder, sstream, &parsed, &errs)) {
      output.clear();
      for (const auto &element : parsed) {
        output.push_back(element.asFloat());
      }
    }
  }
}
