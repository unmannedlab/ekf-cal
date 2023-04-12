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

#ifndef APPLICATION__ROS__NODE__EKFCALNODE_HPP_
#define APPLICATION__ROS__NODE__EKFCALNODE_HPP_


#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ekf/EKF.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "sensors/Camera.hpp"
#include "sensors/IMU.hpp"
#include "sensors/ros/RosCamera.hpp"
#include "sensors/ros/RosIMU.hpp"
#include "trackers/FeatureTracker.hpp"

///
/// @class EkfCalNode
/// @brief A ROS2 node for EKF-based sensor calibration
/// @todo Camera Methods
/// @todo Software Paper
/// @todo Architecture Design
/// @todo Option to publish health metrics
/// @todo Option to publish visualization messages
/// @todo Create generic callback that can be used to store and sort measurements
///
class EkfCalNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the Calibration EKF Node
  ///
  EkfCalNode();

  ///
  /// @brief Initialize EKF calibration node
  ///
  void initialize();

  ///
  /// @brief Load lists of sensors
  ///
  void loadSensors();

  ///
  /// @brief Loading method for IMU sensors
  /// @param imuName Name of IMU to find and load from YAML
  ///
  void loadIMU(std::string imuName);

  ///
  /// @brief Loading method for IMU sensors
  /// @param camName Name of IMU to find and load from YAML
  ///
  void loadCamera(std::string camName);

  ///
  /// @brief Function for declaring and loading IMU parameters
  /// @param imuName Name of parameter structure
  /// @return imuParameters
  ///
  IMU::Parameters getImuParameters(std::string imuName);

  ///
  /// @brief Function for declaring and loading camera parameters
  /// @param cameraName Name of parameter structure
  /// @return cameraParameters
  ///
  Camera::Parameters getCameraParameters(std::string cameraName);

  ///
  /// @brief Declare parameters for all sensors
  ///
  void declareSensors();

  ///
  /// @brief Declare IMU parameters
  /// @param imuName IMU parameter namespace
  ///
  void declareImuParameters(std::string imuName);

  ///
  /// @brief Declare camera parameters
  /// @param cameraName Camera parameter namespace
  ///
  void declareCameraParameters(std::string cameraName);

  ///
  /// @brief Declare tracker PARAMETERS
  /// @param trackerName Tracker parameter namespace
  ///
  void declareTrackerParameters(std::string trackerName);

  ///
  /// @brief Function for declaring and loading tracker parameters
  /// @param trackerName Name of parameter structure
  /// @return trackerParameters
  ///
  FeatureTracker::Parameters getTrackerParameters(std::string trackerName);
  ///
  /// @brief Callback method for IMU sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id);
  ///
  /// @brief Callback method for Camera sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id);

  ///
  /// @brief Register IMU sensor
  /// @param imuPtr IMU sensor shared pointer
  /// @param topic Topic to subscribe
  ///
  void registerImu(std::shared_ptr<RosIMU> imuPtr, std::string topic);
  ///
  /// @brief Register camera sensor
  /// @param camPtr Camera sensor shared pointer
  /// @param topic Topic to subscribe
  ///
  void registerCamera(std::shared_ptr<RosCamera> camPtr, std::string topic);

private:
  /// @brief Vector of subscribers for IMU sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_IMUSubs;

  /// @brief Vector of subscribers for Camera sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_CameraSubs;

  bool m_baseIMUAssigned {false};
  std::vector<std::string> m_ImuList {};
  std::vector<std::string> m_CameraList {};
  std::vector<std::string> m_TrackerList {};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imgPublisher;
  DebugLogger * m_logger = DebugLogger::getInstance();
  std::map<int, std::shared_ptr<RosIMU>> m_mapIMU{};
  std::map<int, std::shared_ptr<RosCamera>> m_mapCamera{};
};

#endif  // APPLICATION__ROS__NODE__EKFCALNODE_HPP_
