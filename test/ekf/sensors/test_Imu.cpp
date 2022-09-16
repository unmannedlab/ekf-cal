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

#include "ekf/sensors/Imu.hpp"

#include "TypeHelper.hpp"
#include "CustomAssertions.hpp"

TEST(test_Imu, Name) {
  Imu::Params params;
  params.name = "IMU_Name";
  Imu imu(params);
  EXPECT_EQ(imu.GetName(), params.name);
}


TEST(test_Imu, Id) {
  Imu::Params params1;
  Imu::Params params2;

  Imu imu1(params1);
  Imu imu2(params2);

  unsigned int id_one = imu1.GetId();
  unsigned int id_two = id_one + 1;

  EXPECT_EQ(imu1.GetId(), id_one);
  EXPECT_EQ(imu2.GetId(), id_two);
}


TEST(test_Imu, StateStartIndex) {
  Imu::Params params;
  Imu imu(params);

  unsigned int index1 = 1U;
  unsigned int index2 = 2U;

  imu.SetStateStartIndex(index1);
  EXPECT_EQ(imu.GetStateStartIndex(), index1);

  imu.SetStateStartIndex(index2);
  EXPECT_EQ(imu.GetStateStartIndex(), index2);
}

TEST(test_Imu, StateSize) {
  Imu::Params params1;
  Imu::Params params2;
  Imu::Params params3;
  Imu::Params params4;

  params1.baseSensor = true;
  params1.intrinsic = false;

  Imu imu1(params1);
  EXPECT_EQ(imu1.GetStateSize(), 0U);

  params2.baseSensor = true;
  params2.intrinsic = true;
  Imu imu2(params2);
  EXPECT_EQ(imu2.GetStateSize(), 6U);

  params3.baseSensor = false;
  params3.intrinsic = false;
  Imu imu3(params3);
  EXPECT_EQ(imu3.GetStateSize(), 6U);

  params4.baseSensor = false;
  params4.intrinsic = true;
  Imu imu4(params4);
  EXPECT_EQ(imu4.GetStateSize(), 12U);
}

TEST(test_Imu, PosOffset) {
  Imu::Params params;
  Imu imu(params);

  Eigen::Vector3d posOff1 = {1.2, -3.4, 5.6};
  Eigen::Vector3d posOff2 = {-9.8, 7.6, -5.4};

  imu.SetPosOffset(posOff1);
  EXPECT_EQ(imu.GetPosOffset(), posOff1);

  imu.SetPosOffset(posOff2);
  EXPECT_EQ(imu.GetPosOffset(), posOff2);
}

TEST(test_Imu, AngOffset) {
  Imu::Params params;
  Imu imu(params);

  Eigen::Quaterniond angOff1 = {1.2, -3.4, 5.6, -7.8};
  Eigen::Quaterniond angOff2 = {-9.8, 7.6, -5.4, 3.2};

  imu.SetAngOffset(angOff1);
  EXPECT_EQ(imu.GetAngOffset(), angOff1);

  imu.SetAngOffset(angOff2);
  EXPECT_EQ(imu.GetAngOffset(), angOff2);
}

TEST(test_Imu, GetAccBiasStability)
{
  Imu::Params params;
  params.accBiasStability = 1.234;
  Imu imu(params);

  EXPECT_EQ(imu.GetAccBiasStability(), params.accBiasStability);
}

TEST(test_Imu, GetOmgBiasStability)
{
  Imu::Params params;
  params.omgBiasStability = 4.567;
  Imu imu(params);

  EXPECT_EQ(imu.GetOmgBiasStability(), params.omgBiasStability);
}

TEST(test_Imu, IsBaseSensor)
{
  Imu::Params params1;
  Imu::Params params2;

  params1.baseSensor = true;
  params2.baseSensor = false;

  Imu imu1(params1);
  Imu imu2(params2);

  EXPECT_TRUE(imu1.IsBaseSensor());
  EXPECT_FALSE(imu2.IsBaseSensor());
}

TEST(test_Imu, IsIntrinsic)
{
  Imu::Params params1;
  Imu::Params params2;

  params1.intrinsic = true;
  params2.intrinsic = false;

  Imu imu1(params1);
  Imu imu2(params2);

  EXPECT_TRUE(imu1.IsIntrinsic());
  EXPECT_FALSE(imu2.IsIntrinsic());
}

TEST(test_Imu, Covariance)
{
  Imu::Params params;
  params.baseSensor = false;
  params.intrinsic = false;
  Imu imu(params);

  EXPECT_EQ(imu.GetStateSize(), 6U);

  Eigen::MatrixXd cov(6, 6);
  cov.setIdentity();

  imu.SetCov(cov);

  EXPECT_EQ(imu.GetCov(), cov);
}

TEST(test_Imu, SetState)
{
  // Base IMU no intrinsics
  Imu::Params params1;
  params1.baseSensor = true;
  params1.intrinsic = false;
  Imu imu1(params1);
  Eigen::VectorXd vecIn1 {};
  imu1.SetState(vecIn1);
  Eigen::VectorXd vecOut1 = imu1.GetState();

  EXPECT_EQ(imu1.GetStateSize(), vecOut1.size());
  EXPECT_EQ(vecOut1, vecIn1);

  // Base IMU with intrinsics
  Imu::Params params2;
  params2.baseSensor = true;
  params2.intrinsic = true;
  Imu imu2(params2);
  Eigen::VectorXd vecIn2(6);
  vecIn2 << 1, -2, 3, -4, 5, -6;
  imu2.SetState(vecIn2);
  Eigen::VectorXd vecOut2 = imu2.GetState();

  EXPECT_EQ(imu2.GetStateSize(), vecOut2.size());
  EXPECT_EQ(vecOut2.segment<3>(0), vecIn2.segment<3>(0));
  EXPECT_NEAR(vecOut2(3), vecIn2(3), 1e-6);
  EXPECT_NEAR(vecOut2(4), vecIn2(4), 1e-6);
  EXPECT_NEAR(vecOut2(5), vecIn2(5), 1e-6);

  // Extrinsic IMU
  Imu::Params params3;
  params3.baseSensor = false;
  params3.intrinsic = false;
  Imu imu3(params3);
  Eigen::VectorXd vecIn3(6);
  vecIn3 << -1, 2, -3, 0.4, -0.5, 0.6;
  imu3.SetState(vecIn3);
  Eigen::VectorXd vecOut3 = imu3.GetState();

  EXPECT_EQ(imu3.GetStateSize(), vecOut3.size());
  EXPECT_EQ(vecOut3.segment<3>(0), vecIn3.segment<3>(0));
  EXPECT_NEAR(vecOut3(3), vecIn3(3), 1e-6);
  EXPECT_NEAR(vecOut3(4), vecIn3(4), 1e-6);
  EXPECT_NEAR(vecOut3(5), vecIn3(5), 1e-6);

  // Intrinsic IMU
  Imu::Params params4;
  params4.baseSensor = false;
  params4.intrinsic = true;
  Imu imu4(params4);
  Eigen::VectorXd vecIn4(12);
  vecIn4 << 1, 2, 3, 0.4, 0.5, 0.6, 7, 8, 9, 10, 11, 12;
  imu4.SetState(vecIn4);
  Eigen::VectorXd vecOut4 = imu4.GetState();

  EXPECT_EQ(imu4.GetStateSize(), vecOut4.size());
  EXPECT_EQ(vecOut4.segment<3>(0), vecIn4.segment<3>(0));
  EXPECT_EQ(vecOut4.segment<3>(6), vecIn4.segment<3>(6));
  EXPECT_EQ(vecOut4.segment<3>(9), vecIn4.segment<3>(9));
  EXPECT_NEAR(vecOut4(3), vecIn4(3), 1e-6);
  EXPECT_NEAR(vecOut4(4), vecIn4(4), 1e-6);
  EXPECT_NEAR(vecOut4(5), vecIn4(5), 1e-6);
}

TEST(test_Imu, PredictMeasurement) {
  Eigen::VectorXd bodyVec(18);
  bodyVec.setZero();
  bodyVec.segment(6, 3) << 1, 2, 3;         // Body acceleration
  bodyVec.segment(12, 3) << 0.4, 0.5, 0.6;  // Body angular velocity
  bodyVec.segment(15, 3) << 0.7, 0.8, 0.9;  // Body angular acceleration
  Sensor::SetBodyState(bodyVec);


  // Base IMU no intrinsics
  Imu::Params params1;
  params1.baseSensor = true;
  params1.intrinsic = false;
  Imu imu1(params1);
  Eigen::VectorXd vecIn1 {};
  imu1.SetState(vecIn1);

  Eigen::VectorXd vecOut1 = imu1.PredictMeasurement();
  EXPECT_EQ(vecOut1.segment(0, 3), bodyVec.segment(6, 3));
  EXPECT_EQ(vecOut1.segment(3, 3), bodyVec.segment(12, 3));


  // Base IMU with intrinsics
  Imu::Params params2;
  params2.baseSensor = true;
  params2.intrinsic = true;
  Imu imu2(params2);
  Eigen::VectorXd vecIn2(6);
  vecIn2 << 1, -2, 3, -4, 5, -6;
  imu2.SetState(vecIn2);

  Eigen::VectorXd vecOut2 = imu2.PredictMeasurement();
  EXPECT_EQ(vecOut2.segment(0, 3), bodyVec.segment(6, 3) + vecIn2.segment(0, 3));
  EXPECT_EQ(vecOut2.segment(3, 3), bodyVec.segment(12, 3) + vecIn2.segment(3, 3));


  // Extrinsic IMU
  Imu::Params params3;
  params3.baseSensor = false;
  params3.intrinsic = false;
  Imu imu3(params3);
  Eigen::VectorXd vecIn3(6);
  vecIn3 << -1, 2, -3, 0.4, -0.5, 0.6;
  imu3.SetState(vecIn3);

  Eigen::VectorXd vecOut3 = imu3.PredictMeasurement();
  Eigen::Vector3d expAcc {-4.946732775610, -3.791386724870, 4.104999579680};
  Eigen::Vector3d expOmg {-0.219656510510, 0.256260965523, 0.809988478276};

  EXPECT_NEAR(vecOut3(0), expAcc(0), 1e-6);
  EXPECT_NEAR(vecOut3(1), expAcc(1), 1e-6);
  EXPECT_NEAR(vecOut3(2), expAcc(2), 1e-6);
  EXPECT_NEAR(vecOut3(3), expOmg(0), 1e-6);
  EXPECT_NEAR(vecOut3(4), expOmg(1), 1e-6);
  EXPECT_NEAR(vecOut3(5), expOmg(2), 1e-6);


  // Intrinsic IMU
  Imu::Params params4;
  params4.baseSensor = false;
  params4.intrinsic = true;
  Imu imu4(params4);
  Eigen::VectorXd vecIn4(12);
  vecIn4 <<  -1, 2, -3, 0.4, -0.5, 0.6, -7, 8, -9, 10, -11, 12;
  imu4.SetState(vecIn4);
  Eigen::VectorXd vecOut4 = imu4.PredictMeasurement();
  EXPECT_NEAR(vecOut4(0), expAcc(0) + vecIn4(6), 1e-6);
  EXPECT_NEAR(vecOut4(1), expAcc(1) + vecIn4(7), 1e-6);
  EXPECT_NEAR(vecOut4(2), expAcc(2) + vecIn4(8), 1e-6);
  EXPECT_NEAR(vecOut4(3), expOmg(0) + vecIn4(9), 1e-6);
  EXPECT_NEAR(vecOut4(4), expOmg(1) + vecIn4(10), 1e-6);
  EXPECT_NEAR(vecOut4(5), expOmg(2) + vecIn4(11), 1e-6);
}


TEST(test_Imu, GetMeasurementJacobian) {
  // Base IMU no intrinsics
  Imu::Params params1;
  params1.baseSensor = true;
  params1.intrinsic = false;
  Imu imu1(params1);
  Eigen::VectorXd vecIn1 {};
  vecIn1.setZero();
  imu1.SetState(vecIn1);

  Eigen::MatrixXd jacOut1(6, 18);
  Eigen::MatrixXd jacExp1(6, 18);

  jacOut1 = imu1.GetMeasurementJacobian();
  jacExp1.setZero();

  jacExp1.block<3, 3>(0, 6).setIdentity();
  jacExp1.block<3, 3>(3, 12).setIdentity();

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(jacOut1, jacExp1, 1e-6));


  // Base IMU with intrinsics
  Imu::Params params2;
  params2.baseSensor = true;
  params2.intrinsic = true;
  Imu imu2(params2);
  Eigen::VectorXd vecIn2(6);
  vecIn2 << -1, 2, -3, 4, -5, 6;
  imu2.SetState(vecIn2);

  Eigen::MatrixXd jacOut2(6, 24);
  Eigen::MatrixXd jacExp2(6, 24);

  jacOut2 = imu2.GetMeasurementJacobian();
  jacExp2.setZero();

  jacExp2.block<3, 3>(0, 6).setIdentity();
  jacExp2.block<3, 3>(3, 12).setIdentity();
  jacExp2.block<3, 3>(0, 18).setIdentity();
  jacExp2.block<3, 3>(3, 21).setIdentity();

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(jacOut2, jacExp2, 1e-6));


  // Extrinsic IMU
  Imu::Params params3;
  params3.baseSensor = false;
  params3.intrinsic = false;
  Imu imu3(params3);
  Eigen::VectorXd vecIn3(6);
  vecIn3 << 1, -2, 3, -0.4, 0.5, -0.6;
  imu3.SetState(vecIn3);
  Eigen::MatrixXd jacOut3(6, 24);
  Eigen::MatrixXd jacExp3(6, 24);

  jacOut3 = imu3.GetMeasurementJacobian();
  jacExp3.setZero();

  jacExp3.block<1, 3>(0, 6) << 0.714075363, 0.432164946, 0.550753879;
  jacExp3.block<1, 3>(1, 6) << -0.619656511, 0.756260966, 0.209988478;
  jacExp3.block<1, 3>(2, 6) << -0.325764001, -0.491225826, 0.807821146;

  jacExp3.block<1, 3>(0, 12) << 0.487449694, -2.64773907, 1.35499096;
  jacExp3.block<1, 3>(1, 12) << 0.714443558, 1.89720423, 2.82342468;
  jacExp3.block<1, 3>(2, 12) << -2.7462635, -3.88717043, -2.40047341;

  jacExp3.block<1, 3>(0, 15) << 2.39800259, -1.59147221, -1.86031567;
  jacExp3.block<1, 3>(1, 15) << 2.68875985, 2.06895801, 0.483052055;
  jacExp3.block<1, 3>(2, 15) << 0.141964815, 1.78511315, 1.14275383;

  jacExp3.block<1, 3>(0, 18) << -0.661654652, -0.0571401118, 0.508744829;
  jacExp3.block<1, 3>(1, 18) << 0.712644086, 0.184682794, -0.725107884;
  jacExp3.block<1, 3>(2, 18) << -0.84836016, 1.1586624, -0.326753123;

  jacExp3.block<1, 3>(0, 21) << 1.96062671, -3.26832108, 0.0225483141;
  jacExp3.block<1, 3>(1, 21) << 1.21481229, -0.541514785, 5.53503148;
  jacExp3.block<1, 3>(2, 21) << 1.98692577, -3.70904827, -1.45417264;

  jacExp3.block<1, 3>(3, 12) << 0.714075363, 0.432164946, 0.550753879;
  jacExp3.block<1, 3>(4, 12) << -0.619656511, 0.756260966, 0.209988478;
  jacExp3.block<1, 3>(5, 12) << -0.325764001, -0.491225826, 0.807821146;

  jacExp3.block<1, 3>(3, 21) << 0.0160779722, 0.208143666, -0.184171703;
  jacExp3.block<1, 3>(4, 21) <<  -0.34876234, -0.455789298, 0.612332641;
  jacExp3.block<1, 3>(5, 21) << 0.698646068, -0.518586859, -0.0336083298;

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(jacOut3, jacExp3, 1e-6));


  // Intrinsic IMU
  Imu::Params params4;
  params4.baseSensor = false;
  params4.intrinsic = true;
  Imu imu4(params4);
  Eigen::VectorXd vecIn4(12);
  vecIn4 << 1, -2, 3, -0.4, 0.5, -0.6, 7, -8, 9, -10, 11, -12;
  imu4.SetState(vecIn4);
  Eigen::MatrixXd jacOut4(6, 30);
  Eigen::MatrixXd jacExp4(6, 30);

  jacOut4 = imu4.GetMeasurementJacobian();
  jacExp4.setZero();

  jacExp4.block<6, 24>(0, 0) = jacExp3;
  jacExp4.block<6, 6>(0, 24).setIdentity();

  EXPECT_TRUE(CustomAssertions::EXPECT_EIGEN_NEAR(jacOut4, jacExp4, 1e-6));
}
