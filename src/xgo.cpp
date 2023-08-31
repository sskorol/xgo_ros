#include "xgo_ros/xgo.h"

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace tf;

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

XGO::XGO(NodeHandle *nodehandle)
    : nh_(*nodehandle), current_time(ros::Time::now()),
      last_time(ros::Time::now()) {
  ROS_INFO("Initializing xgo node...");
  initParam();
  initCOM();
  initSubcriber();
  initPublisher();
}

bool XGO::initParam() {
  if (nh_.getParam("vx_max", vx_max) &&
      nh_.getParam("vy_max", vy_max) &&
      nh_.getParam("vyaw_max", vyaw_max) &&
      nh_.getParam("period_limit", period_limit) &&
      nh_.getParam("joint_limit", joint_limit_xml) &&
      nh_.getParam("arm_limit", arm_limit_xml) &&
      nh_.getParam("body_limit", body_limit_xml) &&
      nh_.getParam("enable_joint_gui", enable_joint_gui)) {
    joint_limit.resize(joint_limit_xml.size());
    for (int i = 0; i < joint_limit_xml.size(); ++i) {
      for (int j = 0; j < joint_limit_xml[i].size(); ++j) {
        joint_limit[i].push_back(static_cast<int>(joint_limit_xml[i][j]));
      }
    }

    arm_limit.resize(arm_limit_xml.size());
    for (int i = 0; i < arm_limit_xml.size(); ++i) {
      for (int j = 0; j < arm_limit_xml[i].size(); ++j) {
        arm_limit[i].push_back(static_cast<int>(arm_limit_xml[i][j]));
      }
    }

    body_limit.resize(body_limit_xml.size());
    for (int i = 0; i < body_limit_xml.size(); ++i) {
      for (int j = 0; j < body_limit_xml[i].size(); ++j) {
        body_limit[i].push_back(static_cast<int>(body_limit_xml[i][j]));
      }
    }
    return true;
  } else {
    ROS_ERROR("Initializing param failed!");
    return false;
  }
}

bool XGO::initCOM() {
  if (nh_.getParam("com_port", com_port_str)) {
    cout << "Connecting to " << com_port_str << endl;
  }
  strcpy(com_port, com_port_str.c_str());
  Timeout to = Timeout::simpleTimeout(100);
  ser.setPort(com_port_str);
  ser.setBaudrate(115200);
  ser.setTimeout(to);
  ser.open();
  sendOrder(0x08, 0x01);
  // Auto-balancing
  // sendOrder(0x61, 0x01);
  return true;
}

void XGO::initSubcriber() {
  sub_cmd_vel_ = nh_.subscribe("cmd_vel", 10, &XGO::cmdvelCallback, this);
  sub_body_pose_ = nh_.subscribe("body_pose", 10, &XGO::bodyposeCallback, this);
  sub_arm_pose_ = nh_.subscribe("arm_pose", 10, &XGO::armposeCallback, this);
  sub_joint_angle_ =
      nh_.subscribe("joint_angle", 10, &XGO::jointangleCallback, this);
  sub_leg_pose_ = nh_.subscribe("leg_pose", 10, &XGO::legposeCallback, this);
  sub_order_ = nh_.subscribe("order", 10, &XGO::orderCallback, this);
}

void XGO::initPublisher() {
  if (!enable_joint_gui) {
    pub_joint_ = nh_.advertise<JointState>("/joint_states", 10);
  }
  pub_imu_ = nh_.advertise<Imu>("/imu/data", 10);
  vel_pub_ = nh_.advertise<Twist>("/cmd_vel", 10);
  battery_pub_ = nh_.advertise<BatteryState>("/battery", 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
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
  pubImuState();
  publishOdometry();
}

void XGO::cmdvelCallback(const Twist &msg) {
  vx = limit(msg.linear.x, -vx_max, vx_max);
  vy = limit(msg.linear.y, -vy_max, vy_max);
  vyaw = limit(msg.angular.z, -vyaw_max, vyaw_max);
  sendSpeed();
}

void XGO::jointangleCallback(const Float32MultiArray &msg) {
  // motor11 motor12 motor13 motor21 .. motor51 motor52 motor53
  for (int i = 0; i < 15; i++) {
    joint_angle[i] = msg.data[i];
    sendMotorAngle();
  }
}

void XGO::bodyposeCallback(const geometry_msgs::Pose &msg) {
  // x y z roll pitch yaw
  geometry_msgs::Point p = msg.position;
  geometry_msgs::Quaternion q = msg.orientation;
  body_pose[0] += p.x;
  body_pose[1] += p.y;
  body_pose[2] += p.z;
  body_pose[3] += q.x;
  body_pose[4] += q.y;
  body_pose[5] += q.z;
  sendBodyPose();
}

void XGO::legposeCallback(const Float32MultiArray &msg) {
  // leg1_x leg1_y leg1_z  leg2_x leg2_y leg2_z ...
  for (int i = 0; i < 12; i++) {
    leg_pose[i] = msg.data[i];
    sendLegPose();
  }
}

void XGO::armposeCallback(const geometry_msgs::Pose &msg) {
  // forward/backward, up/down, left/right
  geometry_msgs::Point p = msg.position;
  arm_pose[0] = limit(arm_pose[0] + p.x, arm_limit[0][0], arm_limit[0][1]);
  arm_pose[1] = limit(arm_pose[1] + p.y, arm_limit[1][0], arm_limit[1][1]);
  arm_pose[2] = limit(arm_pose[2] + p.z, arm_limit[2][0], arm_limit[2][1]);
  sendArmPose();
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

void XGO::orderCallback(const UInt8MultiArray &msg) {
  sendOrder(msg.data[0], msg.data[1], (uint8_t *)msg.data.data() + 2);
}

void XGO::action(uint8_t action_id) { sendOrder(0x3E, action_id); }

void XGO::reset() { action(0x00); }

void XGO::remount() { sendOrder(0x20, 0x00); }

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
    JointState jointState;
    jointState.header.stamp = Time::now();
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
    pub_joint_.publish(jointState);
  }
}

void XGO::pubBaseTf() {
  q.setRPY(imu_angle[0], imu_angle[1], imu_angle[2]);
  ts.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  ts.setRotation(q);
  br.sendTransform(tf::StampedTransform(ts, Time::now(), "odom", "base_link"));
}

void XGO::pubBattery() {
  BatteryState msg;
  msg.percentage = battery;
  battery_pub_.publish(msg);
}

void XGO::pubImuState() {
  Imu imuState;
  imuState.header.stamp = Time::now();
  imuState.header.frame_id = "imu_link";
  geometry_msgs::Quaternion quat;
  q.setRPY(imu_angle[0], imu_angle[1], imu_angle[2]);
  q.normalize();
  tf::quaternionTFToMsg(q, quat);
  imuState.orientation = quat;
  imuState.linear_acceleration.x = imu_acc[0];
  imuState.linear_acceleration.y = imu_acc[1];
  imuState.linear_acceleration.z = imu_acc[2];
  pub_imu_.publish(imuState);
}

void XGO::publishOdometry() {
  current_time = Time::now();

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(imu_angle[2]) - vy * sin(imu_angle[2])) * dt;
  double delta_y = (vx * sin(imu_angle[2]) + vy * cos(imu_angle[2])) * dt;

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x += delta_x;
  odom.pose.pose.position.y += delta_y;
  odom.pose.pose.position.z = 0.12; // considering robot's height, adjust as needed
  odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
      imu_angle[0], imu_angle[1], imu_angle[2]
  );

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vyaw;

  // publish the message
  odom_pub_.publish(odom);

  last_time = current_time;
}