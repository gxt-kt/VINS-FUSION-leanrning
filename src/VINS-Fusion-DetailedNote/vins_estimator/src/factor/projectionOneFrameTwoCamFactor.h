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
#include "fpm.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

class ProjectionOneFrameTwoCamFactor
    : public ceres::SizedCostFunction<2, 7, 7, 1, 1> {
 public:
  ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &_pts_i,
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

class GxtProjectionOneFrameTwoCamFactor : public Edge {
 public:
  GxtProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &_pts_i,
                                    const Eigen::Vector3d &_pts_j,
                                    const Eigen::Vector2d &_velocity_i,
                                    const Eigen::Vector2d &_velocity_j,
                                    const double _td_i, const double _td_j)
      : Edge(2, 1,
             std::vector<std::string>{"VertexInverseDepth", "VertexPose",
                                      "VertexPose"}),
        pts_i(_pts_i),
        pts_j(_pts_j),
        td_i(_td_i),
        td_j(_td_j) {
    velocity_i.x() = _velocity_i.x();
    velocity_i.y() = _velocity_i.y();
    velocity_i.z() = 0;
    velocity_j.x() = _velocity_j.x();
    velocity_j.y() = _velocity_j.y();
    velocity_j.z() = 0;
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if (a == tmp) tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
  }

  /// 计算残差
  virtual void ComputeResidual() override;

  /// 计算雅可比
  virtual void ComputeJacobians() override;

  /// 返回边的类型信息
  virtual std::string TypeInfo() const override { return "EdgeReprojection"; }

  Eigen::Vector3d tic;
  Eigen::Quaterniond qic;
  Eigen::Vector3d tic2;
  Eigen::Quaterniond qic2;
  double td;

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Vector3d velocity_i, velocity_j;
  double td_i, td_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};

