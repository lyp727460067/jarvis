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

#include <Eigen/Dense>
#include <iostream>
#include <map>

#include "jarvis/estimator/feature_manager.h"
#include "jarvis/estimator/factor/imu_factor.h"
#include "jarvis/utility/utility.h"
namespace jarvis {
namespace estimator {
using namespace Eigen;
using namespace std;


class ImageFrame {
 public:
  ImageFrame(){};
  ImageFrame(
      const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points,
      double _t)
      : t{_t}, is_key_frame{false} {
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;
  double t = 0.0;
  Matrix3d R;
  Vector3d T;
  IntegrationBase *pre_integration = nullptr;
  bool is_key_frame = false;
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame,
                        Vector3d *Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs,
                        Vector3d &g, VectorXd &x);
}  // namespace estimator
}  // namespace jarvis