/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <execinfo.h>

#include <Eigen/Dense>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>

#include "jarvis/estimator/parameters.h"
#include "jarvis/utility/tic_toc.h"
#include "camera_models/camera_models/camera_factory.h"
#include "camera_models/camera_models/CataCamera.h"
#include "camera_models/camera_models/PinholeCamera.h"
namespace jarvis {
namespace estimator {
using namespace std;
using namespace camera_models;
using namespace Eigen;
struct ImageFeatureTrackerResult {
  struct FeatureTrackerResult {
    int id;
    struct CameraFeature {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      int id;
      Eigen::Vector3d normal_points;
      Eigen::Vector2d uv;
      Eigen::Vector2d uv_velocity;
    };
    std::vector<CameraFeature> camera_features;
  };
  struct Data {
    double time;
    std::map<int, FeatureTrackerResult> features;
    std::map<int, int> tracker_features_num;
    std::map<int, cv::Mat> images;  // camera_id,image
  };
  std::shared_ptr<Data> data;
};

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker {
 public:
  FeatureTracker();
  ImageFeatureTrackerResult trackImage(double _cur_time, const cv::Mat &_img,
                                       const cv::Mat &_img1 = cv::Mat(),
                                       std::map<int, int> *track_cnt = nullptr);
  void setMask();
  void readIntrinsicParameter(const vector<string> &calib_file);
  void showUndistortion(const string &name);
  void rejectWithF();
  void undistortedPoints();
  vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts,
                                     camera_models::CameraPtr cam);
  vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts,
                                  map<int, cv::Point2f> &cur_id_pts,
                                  map<int, cv::Point2f> &prev_id_pts);
  void showTwoImage(const cv::Mat &img1, const cv::Mat &img2,
                    vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
  void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                 vector<int> &curLeftIds, vector<cv::Point2f> &curLeftPts,
                 vector<cv::Point2f> &curRightPts,
                 map<int, cv::Point2f> &prevLeftPtsMap);
  void setPrediction(map<int, Eigen::Vector3d> &predictPts);
  double distance(cv::Point2f &pt1, cv::Point2f &pt2);
  void removeOutliers(set<int> &removePtsIds);
  cv::Mat getTrackImage();
  bool inBorder(const cv::Point2f &pt);

  int row = 0, col = 0;
  cv::Mat imTrack;
  cv::Mat mask;
  cv::Mat fisheye_mask;
  cv::Mat prev_img, cur_img;
  vector<cv::Point2f> n_pts;
  vector<cv::Point2f> predict_pts;
  vector<cv::Point2f> predict_pts_debug;
  vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
  vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
  vector<cv::Point2f> pts_velocity, right_pts_velocity;
  vector<int> ids, ids_right;
  vector<int> track_cnt;
  map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
  map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
  map<int, cv::Point2f> prevLeftPtsMap;
  vector<camera_models::CameraPtr> m_camera;
  double cur_time = 0.0;
  double prev_time = 0;
  bool stereo_cam = 0;
  int n_id = 0;
  bool hasPrediction = false;
};
}  // namespace estimator
}  // namespace jarvis