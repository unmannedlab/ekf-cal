// Copyright 2022 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "EkfCalNode.hpp"

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "TypeHelper.hpp"
#include "RosHelper.hpp"
#include "ekf/EKF.hpp"
#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare Parameters
  this->declare_parameter("IMU_list");
  this->declare_parameter("Camera_list");
  this->declare_parameter("LIDAR_list");

  // Load lists of sensors
  std::vector<std::string> imuList = this->get_parameter("IMU_list").as_string_array();
  std::vector<std::string> camList = this->get_parameter("Camera_list").as_string_array();
  std::vector<std::string> lidarList = this->get_parameter("LIDAR_list").as_string_array();

  // Load Imu sensor parameters
  for (std::string & imuName : imuList) {
    LoadImu(imuName);
  }
  if (m_baseImuAssigned == false) {
    RCLCPP_WARN(get_logger(), "Base IMU should be set for filter stability");
  }

  // Load Camera sensor parameters
  for (std::string & camName : camList) {
    LoadCamera(camName);
  }

  // Load Lidar sensor parameters
  for (std::string & lidarName : lidarList) {
    LoadLidar(lidarName);
  }

  // Create publishers
  m_PosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  m_TwistPub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
  m_StatePub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", 10);

  m_tfTimer =
    this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&EkfCalNode::PublishTransforms, this));
  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void EkfCalNode::LoadImu(std::string imuName)
{
  // Declare parameters
  std::string imuPrefix = "IMUs." + imuName;
  this->declare_parameter(imuPrefix + ".BaseSensor");
  this->declare_parameter(imuPrefix + ".Intrinsic");
  this->declare_parameter(imuPrefix + ".Rate");
  this->declare_parameter(imuPrefix + ".Topic");

  // Load parameters
  bool baseSensor = this->get_parameter(imuPrefix + ".BaseSensor").as_bool();
  bool intrinsic = this->get_parameter(imuPrefix + ".Intrinsic").as_bool();
  double rate = this->get_parameter(imuPrefix + ".Rate").as_double();
  std::string topic = this->get_parameter(imuPrefix + ".Topic").as_string();
  std::vector<double> posOff {0, 0, 0};
  std::vector<double> angOff {1, 0, 0, 0};
  std::vector<double> accBias {0, 0, 0};
  std::vector<double> omgBias {0, 0, 0};

  // Only calibrate offsets if not the base IMU
  if (baseSensor == false) {
    this->declare_parameter(imuPrefix + ".PosOffInit");
    this->declare_parameter(imuPrefix + ".AngOffInit");
    posOff = this->get_parameter(imuPrefix + ".PosOffInit").as_double_array();
    angOff = this->get_parameter(imuPrefix + ".AngOffInit").as_double_array();
  }

  // Only calibrate intrinsics if flag is set
  if (intrinsic == true) {
    this->declare_parameter(imuPrefix + ".AccBiasInit");
    this->declare_parameter(imuPrefix + ".OmgBiasInit");
    accBias = this->get_parameter(imuPrefix + ".AccBiasInit").as_double_array();
    omgBias = this->get_parameter(imuPrefix + ".OmgBiasInit").as_double_array();
  }

  // Assign parameters to struct
  Imu::Params imuParams;
  imuParams.name = imuName;
  imuParams.baseSensor = baseSensor;
  imuParams.intrinsic = intrinsic;
  imuParams.rate = rate;
  imuParams.posOffset = TypeHelper::StdToEigVec(posOff);
  imuParams.angOffset = TypeHelper::StdToEigQuat(angOff);
  imuParams.accBias = TypeHelper::StdToEigVec(accBias);
  imuParams.omgBias = TypeHelper::StdToEigVec(omgBias);

  if ((baseSensor == false) || (intrinsic == true)) {
    this->declare_parameter(imuPrefix + ".VarInit");
    std::vector<double> variance = this->get_parameter(imuPrefix + ".VarInit").as_double_array();
    imuParams.variance = TypeHelper::StdToEigVec(variance);
  }

  // Register IMU and bind callback to ID
  unsigned int id = m_ekf.RegisterSensor(imuParams);
  std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
  function = std::bind(&EkfCalNode::ImuCallback, this, _1, id);
  m_ImuSubs.push_back(this->create_subscription<sensor_msgs::msg::Imu>(topic, 10, function));

  if (imuParams.baseSensor) {
    m_baseImuAssigned = true;
  }
  RCLCPP_INFO(get_logger(), "Loaded IMU: '%s'", imuName.c_str());
}

void EkfCalNode::LoadCamera(std::string camName)
{
  RCLCPP_INFO(get_logger(), "Camera not Loaded: '%s'", camName.c_str());
}

void EkfCalNode::LoadLidar(std::string lidarName)
{
  RCLCPP_INFO(get_logger(), "LIDAR not Loaded: '%s'", lidarName.c_str());
}

void EkfCalNode::ImuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  unsigned int id)
{
  double time = RosHelper::RosHeaderToTime(msg->header);
  Eigen::Vector3d acc = RosHelper::RosToEigen(msg->linear_acceleration);
  Eigen::Vector3d omg = RosHelper::RosToEigen(msg->angular_velocity);
  Eigen::Matrix3d acc_cov = RosHelper::RosToEigen(msg->linear_acceleration_covariance);
  Eigen::Matrix3d omg_cov = RosHelper::RosToEigen(msg->angular_velocity_covariance);

  m_ekf.ImuCallback(id, time, acc, acc_cov, omg, omg_cov);
  PublishState();
}

void EkfCalNode::CameraCallback()
// void EkfCalNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  // double time = RosHelper::RosHeaderToTime(msg->header);
  m_ekf.CameraCallback();
  // m_ekf.CameraCallback(id, time);
  PublishState();
}

void EkfCalNode::LidarCallback()
// void EkfCalNode::LidarCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg, unsigned int id)
{
  // double time = RosHelper::RosHeaderToTime(msg->header);
  m_ekf.LidarCallback();
  // m_ekf.LidarCallback(id, time);
  PublishState();
}

void EkfCalNode::PublishState()
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  auto state_msg = std_msgs::msg::Float64MultiArray();

  Eigen::VectorXd state = m_ekf.GetState();

  // Position
  pose_msg.pose.position.x = state(0);
  pose_msg.pose.position.y = state(1);
  pose_msg.pose.position.z = state(2);

  // Orientation
  Eigen::Quaterniond quat = TypeHelper::RotVecToQuat(state.segment(9, 3));
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  // Linear Velocity
  twist_msg.twist.linear.x = state(3);
  twist_msg.twist.linear.y = state(4);
  twist_msg.twist.linear.z = state(5);

  // Angular Velocity
  twist_msg.twist.angular.x = state(12);
  twist_msg.twist.angular.y = state(13);
  twist_msg.twist.angular.z = state(14);

  // State msg
  for (unsigned int i = 0; i < state.size(); ++i) {
    state_msg.data.push_back(state(i));
  }

  rclcpp::Time now = this->get_clock()->now();
  pose_msg.header.stamp = now;
  twist_msg.header.stamp = now;

  m_PosePub->publish(pose_msg);
  m_TwistPub->publish(twist_msg);
  m_StatePub->publish(state_msg);
}

///
/// @todo debug issue with future extrapolation in RVIZ
///
void EkfCalNode::PublishTransforms()
{
  std::string baseImuName;
  std::vector<std::string> sensorNames;
  std::vector<Eigen::Vector3d> sensorPosOffsets;
  std::vector<Eigen::Quaterniond> sensorAngOffsets;
  m_ekf.GetTransforms(baseImuName, sensorNames, sensorPosOffsets, sensorAngOffsets);

  // rclcpp::Time now = this->get_clock()->now();

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = baseImuName;

  // Publish Sensor transforms
  for (unsigned int i = 0; i < sensorNames.size(); ++i) {
    // Sensor name
    tf.child_frame_id = sensorNames[i];
    tf.header.stamp = this->get_clock()->now();

    // Sensor position
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    // Sensor Orientation
    /// @todo some of these quaternions are not valid (nan)
    tf.transform.rotation.w = 1.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;

    // Send the transformation
    m_tfBroadcaster->sendTransform(tf);
  }

  Eigen::VectorXd ekfState = m_ekf.GetState();

  // Publish Body transforms
  tf.header.frame_id = "world";
  tf.child_frame_id = baseImuName;
  tf.header.stamp = this->get_clock()->now();

  // Body position
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;

  // Body Orientation
  tf.transform.rotation.w = 1.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;

  m_tfBroadcaster->sendTransform(tf);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfCalNode>());
  rclcpp::shutdown();

  return 0;
}
