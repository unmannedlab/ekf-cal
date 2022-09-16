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

#ifndef EKF__SENSORS__SENSOR_HPP_
#define EKF__SENSORS__SENSOR_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>

///
/// @class Sensor
/// @brief Pure base sensor class
///
class Sensor
{
public:
  ///
  /// @brief Sensor class constructor
  /// @param name Sensor name
  ///
  explicit Sensor(std::string name);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  ///
  virtual Eigen::VectorXd PredictMeasurement() = 0;

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  virtual Eigen::MatrixXd GetMeasurementJacobian() = 0;

  ///
  /// @brief Sensor state setter
  /// @param state Sensor state vector
  ///
  virtual void SetState(Eigen::VectorXd state) = 0;

  ///
  /// @brief Sensor state setter
  /// @param state Sensor state vector
  ///
  virtual Eigen::VectorXd GetState() = 0;

  ///
  /// @brief Sensor state covariance setter method
  /// @param cov Sensor state covariance
  ///
  void SetCov(Eigen::MatrixXd cov);

  ///
  /// @brief Sensor state covariance getter method
  /// @return Sensor state covariance
  ///
  Eigen::MatrixXd GetCov();

  ///
  /// @brief Sensor position offset getter method
  /// @return Position offset vector
  ///
  Eigen::Vector3d GetPosOffset();

  ///
  /// @brief Sensor angular offset getter method
  /// @return Angular offset quaternion
  ///
  Eigen::Quaterniond GetAngOffset();

  ///
  /// @brief Sensor position offset setter method
  /// @param posOffset Position offset vector
  ///
  void SetPosOffset(Eigen::Vector3d posOffset);

  ///
  /// @brief Sensor angular offset setter method
  /// @param angOffset Angular offset quaternion
  ///
  void SetAngOffset(Eigen::Quaterniond angOffset);

  ///
  /// @brief Sensor name getter
  /// @return Sensor name
  ///
  std::string GetName();

  ///
  /// @brief Sensor ID getter method
  /// @return Sensor ID
  ///
  unsigned int GetId();

  ///
  /// @brief Sensor state start index getter method
  /// @return Sensor state start index
  ///
  unsigned int GetStateStartIndex();

  ///
  /// @brief Sensor state size getter method
  /// @return Sensor state size
  ///
  unsigned int GetStateSize();

  ///
  /// @brief Sensor state start setter method
  /// @param stateStartIndex Sensor state start
  ///
  void SetStateStartIndex(unsigned int stateStartIndex);

  ///
  /// @brief Body state setter method
  /// @param bodyState Body state vector
  ///
  static void SetBodyState(Eigen::VectorXd bodyState);

protected:
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};          ///< @brief Sensor position offset vector
  Eigen::Quaterniond m_angOffset{0.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset quaternion

  unsigned int m_stateStartIndex{0};  ///< @brief Sensor state start index
  unsigned int m_stateSize{0};        ///< @brief Sensor state size

  static Eigen::Vector3d m_bodyPos;         ///< @brief Body position vector
  static Eigen::Vector3d m_bodyVel;         ///< @brief Body velocity vector
  static Eigen::Vector3d m_bodyAcc;         ///< @brief Body acceleration vector
  static Eigen::Quaterniond m_bodyAngPos;   ///< @brief Body angular position quaternion
  static Eigen::Vector3d m_bodyAngVel;      ///< @brief Body angular velocity vector
  static Eigen::Vector3d m_bodyAngAcc;      ///< @brief Body angular acceleration vector

  double m_rate;                      ///< @brief Sensor measurement rate
  unsigned int m_id;                  ///< @brief Sensor id
  std::string m_name;                 ///< @brief Sensor name
  Eigen::MatrixXd m_cov;              ///< @brief Sensor state covariance

private:
  static unsigned int _idCount;
};

#endif  // EKF__SENSORS__SENSOR_HPP_
