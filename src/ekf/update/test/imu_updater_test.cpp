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

#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>

#include <string>
#include <iostream>

#include "ekf/constants.hpp"
#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/imu_updater.hpp"

TEST(test_imu_updater, update) {
  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(debug_logger, 10.0, false, "");

  double time_init = 0.0;
  BodyState body_state;
  body_state.m_velocity = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int imu_id{0};
  std::string log_file_directory{""};
  bool data_logging_on {true};

  ImuState imu_state;
  imu_state.is_extrinsic = true;
  imu_state.is_intrinsic = true;
  Eigen::MatrixXd imu_cov = Eigen::MatrixXd::Zero(12, 12);
  ekf->RegisterIMU(imu_id, imu_state, imu_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, log_file_directory, data_logging_on, 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Zero();
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_covariance = Eigen::Matrix3d::Zero();
  bool use_for_prediction {false};

  State state = ekf->GetState();
  EXPECT_EQ(state.m_body_state.m_position[0], 0);
  EXPECT_EQ(state.m_body_state.m_position[1], 0);
  EXPECT_EQ(state.m_body_state.m_position[2], 0);

  double time = time_init + 1;
  imu_updater.UpdateEKF(
    ekf,
    time, acceleration, acceleration_cov, angular_rate, angular_rate_covariance,
    use_for_prediction);

  state = ekf->GetState();
  EXPECT_EQ(state.m_body_state.m_position[0], 1);
  EXPECT_EQ(state.m_body_state.m_position[1], 1);
  EXPECT_EQ(state.m_body_state.m_position[2], 1);

  time += 1;
  imu_updater.UpdateEKF(
    ekf,
    time, acceleration, acceleration_cov, angular_rate, angular_rate_covariance,
    use_for_prediction);

  state = ekf->GetState();
  EXPECT_EQ(state.m_body_state.m_position[0], 2);
  EXPECT_EQ(state.m_body_state.m_position[1], 2);
  EXPECT_EQ(state.m_body_state.m_position[2], 2);
}
