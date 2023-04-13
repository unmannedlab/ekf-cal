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

#include "trackers/sim/SimFeatureTracker.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

SimFeatureTracker::SimFeatureTracker(
  SimFeatureTracker::Parameters params,
  std::shared_ptr<TruthEngine> truthEngine)
: FeatureTracker(params.trackerParams)
{
  m_pxError = params.pxError;
  m_featureCount = params.featureCount;
  m_truth = truthEngine;

  m_featurePoints.push_back(cv::Point3d(1, 0, 0));
  m_featurePoints.push_back(cv::Point3d(-1, 0, 0));
  m_featurePoints.push_back(cv::Point3d(0, 1, 0));
  m_featurePoints.push_back(cv::Point3d(0, -1, 0));
  m_featurePoints.push_back(cv::Point3d(0, 0, 1));
  m_featurePoints.push_back(cv::Point3d(0, 0, -1));
  for (unsigned int i = 0; i < m_featureCount; ++i) {
    cv::Point3d vec;
    vec.x = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec.y = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec.z = m_rng.UniRand(-params.roomSize / 10, params.roomSize / 10);
    m_featurePoints.push_back(vec);
  }

  m_projMatrix = cv::Mat(3, 3, cv::DataType<double>::type, 0.0);
  m_projMatrix.at<double>(0, 0) = m_focalLength;
  m_projMatrix.at<double>(1, 1) = m_focalLength;
  m_projMatrix.at<double>(0, 2) = static_cast<double>(m_imageWidth) / 2.0;
  m_projMatrix.at<double>(1, 2) = static_cast<double>(m_imageHeight) / 2.0;
  m_projMatrix.at<double>(2, 2) = 1;
}

/// @todo Write visibleKeypoints function
std::vector<cv::KeyPoint> SimFeatureTracker::visibleKeypoints(double time)
{
  std::vector<Eigen::Vector3d> keypoints;
  Eigen::Vector3d bodyPos = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond bodyAng = m_truth->GetBodyAngularPosition(time);
  Eigen::Quaterniond camAng = bodyAng * m_angOffset;
  Eigen::Matrix3d camAngEigMat = camAng.toRotationMatrix();
  Eigen::Vector3d camPlaneVec = camAng * Eigen::Vector3d(0, 0, 1);

  cv::Mat camAngCvMat(3, 3, cv::DataType<double>::type);
  camAngCvMat.at<double>(0, 0) = camAngEigMat(0, 0);
  camAngCvMat.at<double>(1, 0) = camAngEigMat(1, 0);
  camAngCvMat.at<double>(2, 0) = camAngEigMat(2, 0);

  camAngCvMat.at<double>(0, 1) = camAngEigMat(0, 1);
  camAngCvMat.at<double>(1, 1) = camAngEigMat(1, 1);
  camAngCvMat.at<double>(2, 1) = camAngEigMat(2, 1);

  camAngCvMat.at<double>(0, 2) = camAngEigMat(0, 2);
  camAngCvMat.at<double>(1, 2) = camAngEigMat(1, 2);
  camAngCvMat.at<double>(2, 2) = camAngEigMat(2, 2);

  // Creating Rodrigues rotation matrix
  cv::Mat rVec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(camAngCvMat, rVec);

  Eigen::Vector3d camPos = bodyPos + bodyAng * m_posOffset;
  cv::Mat T(3, 1, cv::DataType<double>::type);
  T.at<double>(0) = camPos[0];
  T.at<double>(1) = camPos[1];
  T.at<double>(2) = camPos[2];

  // Create zero distortion
  /// @todo grab this from input
  cv::Mat distortion(4, 1, cv::DataType<double>::type);
  distortion.at<double>(0) = 0;
  distortion.at<double>(1) = 0;
  distortion.at<double>(2) = 0;
  distortion.at<double>(3) = 0;

  // Project points
  std::vector<cv::Point2d> projectedPoints;

  /// @todo 2D projection is not correct
  cv::projectPoints(m_featurePoints, rVec, T, m_projMatrix, distortion, projectedPoints);

  // Convert to feature points
  std::vector<cv::KeyPoint> projectedFeatures;
  for (unsigned int i = 0; i < projectedPoints.size(); ++i) {
    cv::Point3d pointCV = m_featurePoints[i];
    Eigen::Vector3d pointEig(pointCV.x, pointCV.y, pointCV.z);

    // Check that point is in front of camera plane
    if (camPlaneVec.dot(pointEig) > 0) {
      cv::KeyPoint feat;
      feat.pt.x = projectedPoints[i].x;
      feat.pt.y = projectedPoints[i].y;
      feat.class_id = i;
      if (
        feat.pt.x > 0 &&
        feat.pt.y > 0 &&
        feat.pt.x < m_imageWidth &&
        feat.pt.y < m_imageHeight)
      {
        projectedFeatures.push_back(feat);
      }
    }
  }

  return projectedFeatures;
}

/// @todo Write generateMessages function
std::vector<std::shared_ptr<SimFeatureTrackerMessage>> SimFeatureTracker::generateMessages(
  std::vector<double> messageTimes, unsigned int sensorID)
{

  m_logger->log(
    LogLevel::INFO, "Generating " + std::to_string(
      messageTimes.size()) + " measurements");

  std::map<unsigned int, std::vector<FeatureTrack>> featureTrackMap;
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> trackerMessages;

  for (unsigned int frameID = 0; frameID < messageTimes.size(); ++frameID) {
    std::vector<std::vector<FeatureTrack>> featureTracks;

    std::vector<cv::KeyPoint> keyPoints = visibleKeypoints(messageTimes[frameID]);

    for (auto & keyPoint :keyPoints) {
      auto featureTrack = FeatureTrack{frameID, keyPoint};
      featureTrackMap[keyPoint.class_id].push_back(featureTrack);
    }

    // Update MSCKF on features no longer detected
    for (auto it = featureTrackMap.cbegin(); it != featureTrackMap.cend(); ) {
      const auto & featureTrack = it->second;
      /// @todo get constant from tracker
      if ((featureTrack.back().frameID < frameID) || (featureTrack.size() >= 20)) {
        // This feature does not exist in the latest frame
        featureTracks.push_back(featureTrack);
        it = featureTrackMap.erase(it);
      } else {
        ++it;
      }
    }
    auto trackerMessage = std::make_shared<SimFeatureTrackerMessage>();
    trackerMessage->featureTracks = featureTracks;
    trackerMessage->time = messageTimes[frameID];
    trackerMessage->trackerID = m_id;
    trackerMessage->sensorID = sensorID;
    trackerMessage->sensorType = SensorType::Tracker;
    trackerMessages.push_back(trackerMessage);
  }
  return trackerMessages;
}

void SimFeatureTracker::callback(
  double time, unsigned int cameraID,
  std::shared_ptr<SimFeatureTrackerMessage> msg)
{
  m_msckfUpdater.updateEKF(time, cameraID, msg->featureTracks);
}