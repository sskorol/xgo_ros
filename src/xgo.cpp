#include "xgo2_ros/conversion_utils.hpp"
#include "xgo2_ros/xgo.hpp"

using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace tf2;
using namespace std;

XGO::XGO() : Node("xgo_control_node")
{
  declare_parameter<string>("config_path", "");
  get_parameter("config_path", config_path);
  config_ = loadConfig();

  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // or RCL_SYSTEM_TIME
  current_time = rclcpp::Time(clock_->now());
  last_time = rclcpp::Time(clock_->now());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  initCOM();
  initSubscribers();
  initPublishers();
}

Config XGO::loadConfig() {
  Json::Value root;
  ifstream configFile(config_path);
  configFile >> root;

  Config config;
  config.comPort = root["com_port"].asString();
  config.vxMax = root["vx_max"].asFloat();
  config.vyMax = root["vy_max"].asFloat();
  config.vYawMax = root["vyaw_max"].asFloat();

  const Json::Value& jointLimitJson = root["joint_limit"];
  for(int i = 0; i < jointLimitJson.size(); i++) {
    vector<int> limit {
      jointLimitJson[i][0].asInt(),
      jointLimitJson[i][1].asInt()
    };
    config.jointLimit.push_back(limit);
  }

  const Json::Value& bodyLimitJson = root["body_limit"];
  for(int i = 0; i < bodyLimitJson.size(); i++) {
    vector<int> limit {
      bodyLimitJson[i][0].asInt(),
      bodyLimitJson[i][1].asInt() 
    };
    config.bodyLimit.push_back(limit);
  }

  const Json::Value& armLimitJson = root["arm_limit"];
  for(int i = 0; i < armLimitJson.size(); i++) {
    vector<int> limit {
      armLimitJson[i][0].asInt(),
      armLimitJson[i][1].asInt() 
    };
    config.armLimit.push_back(limit);
  }

  const Json::Value& periodLimitJson = root["period_limit"];
  for(int i = 0; i < periodLimitJson.size(); i++) {
    config.periodLimit.push_back(periodLimitJson[i].asFloat());
  }

  return config;
}

bool XGO::initCOM()
{
  std::cout << "Connecting to " << config_.comPort << std::endl;
  serial::Timeout serialTimeout = serial::Timeout::simpleTimeout(100);
  serialInterface.setPort(config_.comPort);
  serialInterface.setBaudrate(115200);
  serialInterface.setTimeout(serialTimeout);
  serialInterface.open();
  // Autofeedback mode
  sendOrder(0x08, 0x01);
  return true;
}

void XGO::initSubscribers()
{
  velocitySubscriber = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&XGO::velocityCallback, this, std::placeholders::_1));
  bodyPoseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>(
      "body_pose", 10, std::bind(&XGO::bodyPoseCallback, this, std::placeholders::_1));
  armPoseSubscriber = this->create_subscription<geometry_msgs::msg::Pose>(
      "arm_pose", 10, std::bind(&XGO::armPoseCallback, this, std::placeholders::_1));
}

void XGO::initPublishers()
{
  jointPublisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  imuPublisher = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  batteryPublisher = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery", 10);
  odometryPublisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
}

void XGO::readState()
{
  int readLen = serialInterface.available();

  uint8_t tempPtr = 0;
  if (readLen > 0)
  {
    readLen = serialInterface.read(rxBuffer, readLen);
    while (tempPtr != readLen)
    {
      switch (rxFlag)
      {
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
          if (rxPtr == 40)
          {
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

void XGO::updateState()
{
  battery = rxData[0];
  
  for (int i = 0; i < 15; i++)
  {
    int jointIndex = i < 12 ? i % 3 : i - 9;
    jointAngle[i] = toRad(uint82float(rxData[1 + i], config_.jointLimit[jointIndex][0], config_.jointLimit[jointIndex][1]));
  }

  imuAngle[0] = toRad(bytes2float(rxData + 16));
  imuAngle[1] = toRad(bytes2float(rxData + 20));
  imuAngle[2] = toRad(bytes2float(rxData + 24));
  imuAcceleration[0] = bytes2float(rxData + 28);
  imuAcceleration[1] = bytes2float(rxData + 32);
  imuAcceleration[2] = bytes2float(rxData + 36);
  
  publishJointState();
  publishBaseTransformations();
  publishBattery();
  publishImuState();
  publishOdometry();
}

void XGO::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vx = limit(msg->linear.x, -config_.vxMax, config_.vxMax);
  vy = limit(msg->linear.y, -config_.vyMax, config_.vyMax);
  vyaw = limit(msg->angular.z, -config_.vYawMax, config_.vYawMax);
  sendSpeed();
}

void XGO::bodyPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  // x y z roll pitch yaw
  geometry_msgs::msg::Point p = msg->position;
  geometry_msgs::msg::Quaternion q = msg->orientation;
  bodyPose[0] += p.x;
  bodyPose[1] += p.y;
  bodyPose[2] += p.z;
  bodyPose[3] += q.x;
  bodyPose[4] += q.y;
  bodyPose[5] += q.z;
  sendBodyPose();
}

void XGO::armPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  // forward/backward, up/down, left/right
  geometry_msgs::msg::Point p = msg->position;
  armPose[0] = limit(armPose[0] + p.x, config_.armLimit[0][0], config_.armLimit[0][1]);
  armPose[1] = limit(armPose[1] + p.y, config_.armLimit[1][0], config_.armLimit[1][1]);
  armPose[2] = limit(armPose[2] + p.z, config_.armLimit[2][0], config_.armLimit[2][1]);
  // std::cout << "POS: " << p.x << "; " << p.y << "; " << p.z << std::endl;
  // std::cout << "ARM: " << armPose[0] << "; " << armPose[1] << "; " << armPose[2] << std::endl;
  sendArmPose();
}

void XGO::sendOrder(uint8_t addr, uint8_t data)
{
  OrderPacket packet;
  packet.addr = addr;
  packet.length = 1;
  packet.message[0] = data;
  sendOrder(packet);
}

void XGO::sendOrder(uint8_t addr, uint8_t data_len, uint8_t* data)
{
  OrderPacket packet;
  packet.addr = addr;
  packet.length = data_len;
  for (int i = 0; i < data_len; i++)
  {
    packet.message[i] = data[i];
  }
  sendOrder(packet);
}

void XGO::sendOrder(OrderPacket packet)
{
  uint8_t data[50];
  data[0] = 0x55;
  data[1] = 0x00;
  data[2] = packet.length + 8;
  data[3] = 0x01;
  data[4] = packet.addr;
  data[5 + packet.length] = data[2] + data[3] + data[4];
  for (int i = 0; i < packet.length; i++)
  {
    data[5 + i] = packet.message[i];
    data[5 + packet.length] += data[5 + i];
  }
  data[5 + packet.length] = ~data[5 + packet.length];
  data[6 + packet.length] = 0x00;
  data[7 + packet.length] = 0xAA;
  serialInterface.write(data, 8 + packet.length);
}

void XGO::action(uint8_t action_id)
{
  sendOrder(0x3E, action_id);
}

void XGO::reset()
{
  action(0xFF);
}

void XGO::sendBodyPose()
{
  for (int i = 0; i < 6; i++)
  {
    sendOrder(0x33 + i, float2uint8(bodyPose[i], config_.bodyLimit[i][0], config_.bodyLimit[i][1]));
  }
}

void XGO::sendArmPose()
{
  sendOrder(0x71, float2uint8(armPose[0], config_.armLimit[0][0], config_.armLimit[0][1]));
  sendOrder(0x73, float2uint8(armPose[1], config_.armLimit[1][0], config_.armLimit[1][1]));
  sendOrder(0x74, float2uint8(armPose[2], config_.armLimit[2][0], config_.armLimit[2][1]));
}

void XGO::sendSpeed()
{
  sendOrder(0x30, float2uint8(vx, -config_.vxMax, config_.vxMax));
  sendOrder(0x31, float2uint8(vy, -config_.vyMax, config_.vyMax));
  sendOrder(0x32, float2uint8(vyaw, -config_.vYawMax, config_.vYawMax));
}

void XGO::publishJointState()
{
  sensor_msgs::msg::JointState jointState;
  jointState.header.stamp = rclcpp::Clock().now();
  jointState.name = {
    "left_front_hip_joint", "left_front_thigh_joint", "left_front_calf_joint",
    "right_front_hip_joint", "right_front_thigh_joint", "right_front_calf_joint",
    "right_back_hip_joint", "right_back_thigh_joint", "right_back_calf_joint",
    "left_back_hip_joint", "left_back_thigh_joint", "left_back_calf_joint",
    "down_arm_joint", "up_arm_joint", "gear_arm_joint", "right_claw_joint", "left_claw_joint"
  };
  jointState.position = { jointAngle[2],          jointAngle[1],  jointAngle[0],  -jointAngle[5],
                          jointAngle[4],          jointAngle[3],  -jointAngle[8], jointAngle[7],
                          jointAngle[6],          jointAngle[11], jointAngle[10], jointAngle[9],
                          -jointAngle[14],        jointAngle[13], jointAngle[12], jointAngle[12] / 110.0,
                          -jointAngle[12] / 110.0
  };
  jointPublisher->publish(jointState);
}

void XGO::publishImuState()
{
  sensor_msgs::msg::Imu imuState;

  imuState.header.stamp = rclcpp::Clock().now();
  imuState.header.frame_id = "imu_link";

  tf2::Quaternion orientation;
  orientation.setRPY(imuAngle[0], imuAngle[1], imuAngle[2]);  // roll, pitch, yaw
  orientation.normalize();

  imuState.orientation.x = orientation.x();
  imuState.orientation.y = orientation.y();
  imuState.orientation.z = orientation.z();
  imuState.orientation.w = orientation.w();

  imuState.linear_acceleration.x = imuAcceleration[0];
  imuState.linear_acceleration.y = imuAcceleration[1];
  imuState.linear_acceleration.z = imuAcceleration[2];

  imuPublisher->publish(imuState);
}

void XGO::publishBaseTransformations()
{
  tf2::Quaternion q;
  q.setRPY(imuAngle[0], imuAngle[1], imuAngle[2]);
  q.normalize();

  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp = this->get_clock()->now();
  ts.header.frame_id = "odom";
  ts.child_frame_id = "base_link";

  ts.transform.translation.x = odom.pose.pose.position.x;
  ts.transform.translation.y = odom.pose.pose.position.y;
  ts.transform.translation.z = odom.pose.pose.position.z;

  ts.transform.rotation.x = q.x();
  ts.transform.rotation.y = q.y();
  ts.transform.rotation.z = q.z();
  ts.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(ts);
}

void XGO::publishBattery()
{
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = battery;
  batteryPublisher->publish(msg);
}

void XGO::publishOdometry()
{
  current_time = clock_->now();

  double dt = (current_time - last_time).seconds();
  double delta_x = (vx * cos(imuAngle[2]) - vy * sin(imuAngle[2])) * dt;
  double delta_y = (vx * sin(imuAngle[2]) + vy * cos(imuAngle[2])) * dt;

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x += delta_x;
  odom.pose.pose.position.y += delta_y;
  odom.pose.pose.position.z = 0.12;

  tf2::Quaternion q;
  q.setRPY(imuAngle[0], imuAngle[1], imuAngle[2]);
  q.normalize();

  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vyaw;

  // publish the message
  odometryPublisher->publish(odom);

  last_time = current_time;
}
