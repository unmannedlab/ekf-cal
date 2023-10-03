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


#ifndef SENSORS__SIM__SIM_IMU_HPP_
#define SENSORS__SIM__SIM_IMU_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/imu.hpp"
#include "sensors/sensor.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @class SimIMU
/// @brief Simulated IMU Sensor Class
///
class SimIMU : public IMU
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    bool no_errors {false};                          ///< @brief Perfect measurements flag
    double time_error {0.0};                         ///< @brief Time offset error
    double time_bias_error {0.0};                    ///< @brief Time offset bias
    double time_skew_error {0.0};                    ///< @brief Time offset error
    Eigen::Vector3d acc_error {0.0, 0.0, 0.0};       ///< @brief Acceleration error
    Eigen::Vector3d omg_error {0.0, 0.0, 0.0};       ///< @brief Angular rate error
    Eigen::Vector3d pos_error {0.0, 0.0, 0.0};       ///< @brief Position offset error
    Eigen::Vector3d ang_error {0.0, 0.0, 0.0};       ///< @brief Angular offset error
    Eigen::Vector3d acc_bias_error {0.0, 0.0, 0.0};  ///< @brief Acceleration bias
    Eigen::Vector3d omg_bias_error {0.0, 0.0, 0.0};  ///< @brief Angular rate bias
    IMU::Parameters imu_params;                      ///< @brief IMU sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimIMU(SimIMU::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated IMU messages
  /// @param max_time Maximum time of generated messages
  ///
  std::vector<std::shared_ptr<SimImuMessage>> GenerateMessages(double max_time);

private:
  double m_time_error{0.0};
  double m_time_bias_error{0.0};
  double m_time_skew_error{0.0};
  Eigen::Vector3d m_acc_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_ang_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_acc_bias_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_bias_error{0.0, 0.0, 0.0};
  Eigen::Vector3d m_acc_bias_true{0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_bias_true{0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_i_in_b_true{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_i_to_b_true{1.0, 0.0, 0.0, 0.0};
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
};


#endif  // SENSORS__SIM__SIM_IMU_HPP_
