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
  virtual void ComputeResidual() override {
    TicToc tic_toc;

    Eigen::Vector2d residuals_tmp;
    double *residuals = &residuals_tmp(0);

    // Vec7 para1 = verticies_[0]->Parameters();  // vertex1
    // Vec7 para2 = verticies_[1]->Parameters();  // vertex2
    Vec1 para3 = verticies_.at(0)->Parameters();  // inverse deep

    // // 优化变量：外参（左目），外参（右目），特征点逆深度，相机与IMU时差
    // Eigen::Vector3d tic(parameters[0][0], parameters[0][1],
    // parameters[0][2]); // t_i_lc Eigen::Quaterniond qic(parameters[0][6],
    // parameters[0][3], parameters[0][4], parameters[0][5]);

    // Eigen::Vector3d tic2(parameters[1][0], parameters[1][1],
    // parameters[1][2]); // t_i_rc Eigen::Quaterniond qic2(parameters[1][6],
    // parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_dep_i = para3(0);
    // double inv_dep_i = parameters[2][0];

    // double td = parameters[3][0];

    // 匹配点ij,
    // 归一化相机点时差校正，对应到采集时刻的位置，因为IMU数据是对应图像采集时刻的
    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    pts_j_td = pts_j - (td - td_j) * velocity_j;

    // 左目归一化相机点
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;  // t_lc_p
    // 左目IMU点
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;  // t_i_p
    Eigen::Vector3d pts_imu_j = pts_imu_i;                 // t_i_p
    Eigen::Vector3d pts_camera_j =
        qic2.inverse() * (pts_imu_j - tic2);          // t_rc_p
    Eigen::Map<Eigen::Vector2d> residual(residuals);  // map 2d

#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    double dep_j = pts_camera_j.z();  // t_rc_p.z()
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif

    residual = sqrt_info * residual;

    // gxt: 赋值残差
    residual_ = residual;
  }

  /// 计算雅可比
  virtual void ComputeJacobians() override {
    TicToc tic_toc;

    double jacobians[4][40];

    Eigen::Vector2d residuals_tmp;
    double *residuals = &residuals_tmp(0);

    // Vec7 para1 = verticies_[0]->Parameters();  // vertex1
    // Vec7 para2 = verticies_[1]->Parameters();  // vertex2
    Vec1 para3 = verticies_.at(0)->Parameters();  // inverse deep

    // // 优化变量：外参（左目），外参（右目），特征点逆深度，相机与IMU时差
    // Eigen::Vector3d tic(parameters[0][0], parameters[0][1],
    // parameters[0][2]); // t_i_lc Eigen::Quaterniond qic(parameters[0][6],
    // parameters[0][3], parameters[0][4], parameters[0][5]);

    // Eigen::Vector3d tic2(parameters[1][0], parameters[1][1],
    // parameters[1][2]); // t_i_rc Eigen::Quaterniond qic2(parameters[1][6],
    // parameters[1][3], parameters[1][4], parameters[1][5]);

    double inv_dep_i = para3(0);
    // double inv_dep_i = parameters[2][0];

    // double td = parameters[3][0];

    // 匹配点ij,
    // 归一化相机点时差校正，对应到采集时刻的位置，因为IMU数据是对应图像采集时刻的
    Eigen::Vector3d pts_i_td, pts_j_td;
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    pts_j_td = pts_j - (td - td_j) * velocity_j;

    // 左目归一化相机点
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;  // t_lc_p
    // 左目IMU点
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;  // t_i_p
    Eigen::Vector3d pts_imu_j = pts_imu_i;                 // t_i_p
    Eigen::Vector3d pts_camera_j =
        qic2.inverse() * (pts_imu_j - tic2);          // t_rc_p
    Eigen::Map<Eigen::Vector2d> residual(residuals);  // map 2d

#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    double dep_j = pts_camera_j.z();  // t_rc_p.z()
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif

    residual = sqrt_info * residual;

    // gxt: 赋值残差
    residual_ = residual;

    if (jacobians) {
      Eigen::Matrix3d ric = qic.toRotationMatrix();
      Eigen::Matrix3d ric2 = qic2.toRotationMatrix();
      Eigen::Matrix<double, 2, 3> reduce(2, 3);
#ifdef UNIT_SPHERE_ERROR
      double norm = pts_camera_j.norm();
      Eigen::Matrix3d norm_jaco;
      double x1, x2, x3;
      x1 = pts_camera_j(0);
      x2 = pts_camera_j(1);
      x3 = pts_camera_j(2);
      norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
          -x1 * x3 / pow(norm, 3), -x1 * x2 / pow(norm, 3),
          1.0 / norm - x2 * x2 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
          -x1 * x3 / pow(norm, 3), -x2 * x3 / pow(norm, 3),
          1.0 / norm - x3 * x3 / pow(norm, 3);
      reduce = tangent_base * norm_jaco;
#else
      reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0,
          1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);
#endif
      reduce = sqrt_info * reduce;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_ex_pose(jacobians[0]);
        Eigen::Matrix<double, 3, 6> jaco_ex;
        jaco_ex.leftCols<3>() = ric2.transpose();
        jaco_ex.rightCols<3>() =
            ric2.transpose() * ric * -Utility::skewSymmetric(pts_camera_i);
        jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
        jacobian_ex_pose.rightCols<1>().setZero();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_ex_pose1(jacobians[1]);
        Eigen::Matrix<double, 3, 6> jaco_ex;
        jaco_ex.leftCols<3>() = -ric2.transpose();
        jaco_ex.rightCols<3>() = Utility::skewSymmetric(pts_camera_j);
        jacobian_ex_pose1.leftCols<6>() = reduce * jaco_ex;
        jacobian_ex_pose1.rightCols<1>().setZero();
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[2]);
#if 1
        jacobian_feature = reduce * ric2.transpose() * ric * pts_i * -1.0 /
                           (inv_dep_i * inv_dep_i);
#else
        jacobian_feature =
            reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
#endif
        jacobians_.at(0) = jacobian_feature;
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[3]);
        jacobian_td =
            reduce * ric2.transpose() * ric * velocity_i / inv_dep_i * -1.0 +
            sqrt_info * velocity_j.head(2);
      }
    }
    sum_t += tic_toc.toc();
  }

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
