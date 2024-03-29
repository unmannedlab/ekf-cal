// Copyright 2023 Jacob Hartzer
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

#ifndef TRACKERS__FEATURE_TRACKER_HPP_
#define TRACKERS__FEATURE_TRACKER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/msckf_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/types.hpp"

///
/// @class FeatureTracker
/// @brief FeatureTracker Class
///
class FeatureTracker
{
public:
  ///
  /// @brief Detector Enumerations
  ///
  enum class FeatureDetectorEnum
  {
    BRISK,
    FAST,
    GFTT,
    MSER,
    ORB,
    SIFT,
  };

  ///
  /// @brief Descriptor Enumerations
  ///
  enum class DescriptorExtractorEnum
  {
    ORB,
    SIFT
  };

  ///
  /// @brief Matcher Enumerations
  ///
  enum class DescriptorMatcherEnum
  {
    BRUTE_FORCE,
    FLANN
  };

  ///
  /// @brief Feature Tracker Initialization parameters structure
  ///
  typedef struct Parameters
  {
    std::string name {""};                                          ///< @brief Feature Tracker name
    FeatureDetectorEnum detector {FeatureDetectorEnum::ORB};            ///< @brief Detector
    DescriptorExtractorEnum descriptor {DescriptorExtractorEnum::ORB};  ///< @brief Descriptor
    DescriptorMatcherEnum matcher {DescriptorMatcherEnum::FLANN};       ///< @brief Matcher
    double threshold {20.0};                                            ///< @brief Threshold
    int sensor_id{-1};                    ///< @brief Associated sensor ID
    std::string output_directory {""};    ///< @brief Feature Tracker data logging directory
    bool data_logging_on {false};         ///< @brief Feature Tracker data logging flag
    double px_error{1e-9};                ///< @brief Pixel error standard deviation
    Intrinsics intrinsics;                ///< @brief Camera intrinsic parameters
    unsigned int min_track_length{2U};    ///< @brief Minimum track length to consider
    unsigned int max_track_length{20U};   ///< @brief Maximum track length before forced output
    double data_log_rate {0.0};           ///< @brief Data logging rate
    double min_feat_dist {1.0};           ///< @brief Minimum feature distance to consider
    std::shared_ptr<DebugLogger> logger;  ///< @brief Debug logger
    std::shared_ptr<EKF> ekf;             ///< @brief EKF to update
  } Parameters;

  ///
  /// @brief FeatureTracker sensor constructor
  /// @param params Parameter struct for feature tracker
  ///
  explicit FeatureTracker(FeatureTracker::Parameters params);

  ///
  /// @brief Down sample features to grid
  /// @param key_points Key points to down sample
  /// @param rows Number of final rows to consider
  /// @param cols Number of final columns to consider
  /// @return Down sampled key points
  ///
  std::vector<cv::KeyPoint> GridFeatures(
    std::vector<cv::KeyPoint> key_points,
    unsigned int rows,
    unsigned int cols);

  ///
  /// @brief Perform track on new image frame
  /// @param time Frame time
  /// @param frame_id Frame ID
  /// @param img_in Input frame
  /// @param img_out Output frame with drawn track lines
  ///
  void Track(double time, int frame_id, cv::Mat & img_in, cv::Mat & img_out);

  ///
  /// @brief Tracker ID getter method
  /// @return Tracker ID
  ///
  unsigned int GetID();

protected:
  unsigned int m_max_track_length;        ///< @brief Maximum track length before forced output
  unsigned int m_min_track_length;        ///< @brief Minimum track length to consider
  MsckfUpdater m_msckf_updater;           ///< @brief MSCKF updater object
  int m_camera_id{-1};                    ///< @brief Associated camera ID of tracker
  unsigned int m_id;                      ///< @brief Tracker ID
  std::shared_ptr<EKF> m_ekf;             ///< @brief EKF
  std::shared_ptr<DebugLogger> m_logger;  ///< @brief Debug logger

private:
  cv::Ptr<cv::FeatureDetector> InitFeatureDetector(
    FeatureDetectorEnum detector,
    double threshold);
  cv::Ptr<cv::DescriptorExtractor> InitDescriptorExtractor(
    DescriptorExtractorEnum extractor,
    double threshold);
  cv::Ptr<cv::DescriptorMatcher> InitDescriptorMatcher(
    DescriptorMatcherEnum matcher);

  cv::Ptr<cv::FeatureDetector> m_feature_detector;
  cv::Ptr<cv::DescriptorExtractor> m_descriptor_extractor;
  cv::Ptr<cv::DescriptorMatcher> m_descriptor_matcher;

  std::vector<cv::KeyPoint> m_prev_key_points;
  std::vector<cv::KeyPoint> m_curr_key_points;
  cv::Mat m_prev_descriptors;
  cv::Mat m_curr_descriptors;

  std::map<unsigned int, std::vector<FeaturePoint>> m_feature_track_map;

  unsigned int GenerateFeatureID();


  double m_px_error;
  static unsigned int m_tracker_count;
};

#endif  // TRACKERS__FEATURE_TRACKER_HPP_
