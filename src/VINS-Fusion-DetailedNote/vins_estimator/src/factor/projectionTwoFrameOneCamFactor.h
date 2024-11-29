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

#include <ceres/ceres.h>
#include <ros/assert.h>

#include <Eigen/Dense>

#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

/**
 * 单目，两帧建立重投影误差
 * 残差维度2（归一化相机平面），优化变量：前一帧位姿7，当前帧位姿7，相机与IMU外参7，特征点逆深度1，相机与IMU时差1
 */
class ProjectionTwoFrameOneCamFactor
    : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {
 public:
  ProjectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i,
                                 const Eigen::Vector3d &_pts_j,
                                 const Eigen::Vector2d &_velocity_i,
                                 const Eigen::Vector2d &_velocity_j,
                                 const double _td_i, const double _td_j);
  virtual bool Evaluate(double const *const *parameters, double *residuals,
                        double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Vector3d velocity_i, velocity_j;
  double td_i, td_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};

#include "problem.h"

class GxtProjectionTwoFrameOneCamFactor : public Edge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  GxtProjectionTwoFrameOneCamFactor(const Eigen::Vector3d &_pts_i,
                                    const Eigen::Vector3d &_pts_j,
                                    const Eigen::Vector2d &_velocity_i,
                                    const Eigen::Vector2d &_velocity_j,
                                    const double _td_i, const double _td_j)
      : Edge(2, 3,
             std::vector<std::string>{"VertexInverseDepth", "VertexPose",
                                      "VertexPose"}),
        pts_i(_pts_i),
        pts_j(_pts_j),
        td_i(_td_i),
        td_j(_td_j) {
    // 两帧的速度（归一化相机平面上）
    velocity_i.x() = _velocity_i.x();
    velocity_i.y() = _velocity_i.y();
    velocity_i.z() = 0;
    velocity_j.x() = _velocity_j.x();
    velocity_j.y() = _velocity_j.y();
    velocity_j.z() = 0;
  }
  // virtual bool Evaluate(double const *const *parameters, double *residuals,
  // double **jacobians) const; void check(double **parameters);

  /// 计算残差
  virtual void ComputeResidual() override;

  /// 计算雅可比
  virtual void ComputeJacobians() override;

  /// 返回边的类型信息
  virtual std::string TypeInfo() const override { return "EdgeReprojection"; }

  Eigen::Vector3d pts_i, pts_j;

  Eigen::Vector3d tic;
  Eigen::Quaterniond qic;
  double td;

  Eigen::Vector3d velocity_i, velocity_j;
  double td_i, td_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
