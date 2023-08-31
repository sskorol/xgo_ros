#ifndef __XGO_H
#define __XGO_H

#include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  uint8_t addr;
  uint8_t length;
  uint8_t message[40];
} OrderPacket;

using namespace ros;
using namespace std;
using namespace serial;
using namespace XmlRpc;
using namespace tf;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace nav_msgs;

class XGO {
public:
  XGO(NodeHandle *nodehandle);
  void readState();

private:
  NodeHandle nh_;
  bool enable_joint_gui;
  XmlRpcValue joint_limit_xml;
  XmlRpcValue body_limit_xml;
  XmlRpcValue arm_limit_xml;
  string com_port_str;
  char com_port[30];
  uint8_t rxFlag = 0;
  uint8_t rxBuffer[256];
  uint8_t rxData[256];
  uint8_t rxPtr = 0;

  Subscriber sub_cmd_vel_;
  Subscriber sub_body_pose_;
  Subscriber sub_arm_pose_;
  Subscriber sub_joint_angle_;
  Subscriber sub_leg_pose_;
  Subscriber sub_order_;
  Serial ser;
  Publisher pub_joint_;
  Publisher pub_imu_;
  Publisher vel_pub_;
  Publisher battery_pub_;
  Publisher odom_pub_;
  Time current_time, last_time;

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

  TransformBroadcaster br;
  tf::Transform ts;
  tf::Quaternion q;
  Odometry odom;

  vector<float> period_limit;
  vector<vector<int>> joint_limit;
  vector<vector<int>> body_limit;
  vector<vector<int>> arm_limit;

  void sendSpeed();
  void sendOrder(uint8_t addr, uint8_t data);
  void sendOrder(uint8_t addr, uint8_t data_len, uint8_t *data);
  void action(uint8_t action_id);
  void reset();
  void remount();
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
  void initSubcriber();
  void initPublisher();
  void sendOrder(OrderPacket packet);
  void cmdvelCallback(const Twist &msg);
  void legposeCallback(const Float32MultiArray &msg);
  void bodyposeCallback(const geometry_msgs::Pose &msg);
  void jointangleCallback(const Float32MultiArray &msg);
  void armposeCallback(const geometry_msgs::Pose &msg);
  void orderCallback(const UInt8MultiArray &msg);
};

float limit(float data, float min_limit, float max_limit);
uint8_t float2uint8(float data, float min_limit, float max_limit);
float uint82float(uint8_t data, float min_limit, float max_limit);
float toRad(float value);

#endif
