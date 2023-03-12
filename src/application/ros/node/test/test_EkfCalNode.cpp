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

#include <gtest/gtest.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "sensors/IMU.hpp"
#include "sensors/ros/RosIMU.hpp"
#include "application/ros/node/EkfCalNode.hpp"

/// @todo Write these integration tests
class test_EkfCalNode : public ::testing::Test
{
protected:
  virtual void SetUp() {rclcpp::init(0, NULL);}
  virtual void TearDown() {rclcpp::shutdown();}
};

TEST_F(test_EkfCalNode, hello_world) {
  EkfCalNode node;

  node.set_parameter(rclcpp::Parameter("Debug_Log_Level", 1));
  node.set_parameter(rclcpp::Parameter("IMU_list", std::vector<std::string>{"TestImu"}));
  node.set_parameter(rclcpp::Parameter("Camera_list", std::vector<std::string>{"TestCamera"}));
  node.set_parameter(rclcpp::Parameter("Tracker_list", std::vector<std::string>{"TestTracker"}));

  node.initialize();
  node.declareSensors();

  node.set_parameter(rclcpp::Parameter("IMU.TestImu.BaseSensor", true));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Intrinsic", false));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Rate", 400.0));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Topic", "/ImuTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.VarInit",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.PosOffInit",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.AngOffInit",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.AccBiasInit", std::vector<double>{0.0, 0.0,
        0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.OmgBiasInit", std::vector<double>{0.0, 0.0,
        0.0}));

  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Rate", 5.0));
  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Topic", "/CameraTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.PosOffInit",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.AngOffInit",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.VarInit",
      std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Tracker", "TestTracker"));

  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.FeatureDetector", 4));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DescriptorExtractor", 0));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DescriptorMatcher", 0));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DetectorThreshold", 10.0));

  node.loadSensors();
  auto imuMsg = std::make_shared<sensor_msgs::msg::Imu>();
  unsigned int imuID = 1;
  imuMsg->header.stamp.sec = 0;
  imuMsg->header.stamp.nanosec = 0;
  imuMsg->linear_acceleration.x = 0.0;
  imuMsg->linear_acceleration.y = 0.0;
  imuMsg->linear_acceleration.z = 0.0;
  imuMsg->angular_velocity.x = 0.0;
  imuMsg->angular_velocity.y = 0.0;
  imuMsg->angular_velocity.z = 0.0;
  imuMsg->linear_acceleration_covariance.fill(0);
  imuMsg->angular_velocity_covariance.fill(0);

  node.imuCallback(imuMsg, imuID);
  EXPECT_TRUE(true);

  imuMsg->header.stamp.nanosec = 500000000;
  node.imuCallback(imuMsg, imuID);
  EXPECT_TRUE(true);

  /// @todo add image to callback
  // sensor_msgs::msg::Image::SharedPtr camMsg{};
  // unsigned int camID = 2;
  // node.cameraCallback(camMsg, camID);

  EXPECT_TRUE(true);
}
