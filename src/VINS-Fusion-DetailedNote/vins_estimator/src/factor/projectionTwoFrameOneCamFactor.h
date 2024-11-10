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
  virtual void ComputeResidual() override {
    Eigen::Vector2d residuals_tmp;
    double *residuals = &residuals_tmp(0);

    // Eigen::Matrix<double, 2, 15> jacobians_tmp;

    // 优化变量：前一帧位姿7，当前位姿7，相机与IMU外参7，特征点逆深度1，相机与IMU时差1

    Vec7 para1 = verticies_[0]->Parameters();  // vertex1
    Vec7 para2 = verticies_[1]->Parameters();  // vertex2
    Vec1 para3 = verticies_[2]->Parameters();  // inverse deep

    Eigen::Vector3d Pi(para1(0), para1(1), para1(2));
    Eigen::Quaterniond Qi(para1(6), para1(3), para1(4), para1(5));

    Eigen::Vector3d Pj(para2(0), para2(1), para2(2));
    Eigen::Quaterniond Qj(para2(6), para2(3), para2(4), para2(5));

    double inv_dep_i = para3(0);
    // Eigen::Vector3d Pi(parameters[0][0], parameters[0][1],
    // parameters[0][2]); Eigen::Quaterniond Qi(parameters[0][6],
    // parameters[0][3],
    //                       parameters[0][4], parameters[0][5]);

    // Eigen::Vector3d Pj(parameters[1][0], parameters[1][1],
    // parameters[1][2]); Eigen::Quaterniond Qj(parameters[1][6],
    // parameters[1][3],
    //                       parameters[1][4], parameters[1][5]);

    // Eigen::Vector3d tic(parameters[2][0], parameters[2][1],
    // parameters[2][2]); Eigen::Quaterniond qic(parameters[2][6],
    // parameters[2][3],
    //                        parameters[2][4], parameters[2][5]);

    // double inv_dep_i = parameters[3][0];

    // double td = parameters[4][0];

    Eigen::Vector3d pts_i_td, pts_j_td;
    // 匹配点ij,
    // 归一化相机点时差校正，对应到采集时刻的位置，因为IMU数据是对应图像采集时刻的
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    pts_j_td = pts_j - (td - td_j) * velocity_j;
    // 转到相机系
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;  // t_ci_p
    // 转到IMU系
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;  // t_i_p
    // 转到世界系
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;  // t_w_p
    // 转到j的IMU系
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);  // t_ji_p
    // 转到j的相机系
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);  // t_jc_p
    Eigen::Map<Eigen::Vector2d> residual(residuals);

#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    // 计算归一化相机平面上的两点误差
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif

    residual = sqrt_info * residual;

    // gxt: 赋值残差
    residual_ = residual;
  };

  /// 计算雅可比
  virtual void ComputeJacobians() override {
    Eigen::Vector2d residuals_tmp;
    double *residuals = &residuals_tmp(0);

    Eigen::Matrix<double, 2, 15> jacobians_tmp;
    double jacobians[5][40];
    // Eigen::Map<Eigen::Matrix<double, 2, 15, Eigen::RowMajor>>
    // jacobians(array[0], 2, 15); jacobians = jacobians_tmp; double *residuals
    // = &residuals_tmp(0);

    // 优化变量：前一帧位姿7，当前位姿7，相机与IMU外参7，特征点逆深度1，相机与IMU时差1
    TicToc tic_toc;

    Vec7 para1 = verticies_[0]->Parameters();  // vertex1
    Vec7 para2 = verticies_[1]->Parameters();  // vertex2
    Vec1 para3 = verticies_[2]->Parameters();  // inverse deep

    Eigen::Vector3d Pi(para1(0), para1(1), para1(2));
    Eigen::Quaterniond Qi(para1(6), para1(3), para1(4), para1(5));

    Eigen::Vector3d Pj(para2(0), para2(1), para2(2));
    Eigen::Quaterniond Qj(para2(6), para2(3), para2(4), para2(5));

    double inv_dep_i = para3(0);
    // Eigen::Vector3d Pi(parameters[0][0], parameters[0][1],
    // parameters[0][2]); Eigen::Quaterniond Qi(parameters[0][6],
    // parameters[0][3],
    //                       parameters[0][4], parameters[0][5]);

    // Eigen::Vector3d Pj(parameters[1][0], parameters[1][1],
    // parameters[1][2]); Eigen::Quaterniond Qj(parameters[1][6],
    // parameters[1][3],
    //                       parameters[1][4], parameters[1][5]);

    // Eigen::Vector3d tic(parameters[2][0], parameters[2][1],
    // parameters[2][2]); Eigen::Quaterniond qic(parameters[2][6],
    // parameters[2][3],
    //                        parameters[2][4], parameters[2][5]);

    // double inv_dep_i = parameters[3][0];

    // double td = parameters[4][0];

    Eigen::Vector3d pts_i_td, pts_j_td;
    // 匹配点ij,
    // 归一化相机点时差校正，对应到采集时刻的位置，因为IMU数据是对应图像采集时刻的
    pts_i_td = pts_i - (td - td_i) * velocity_i;
    pts_j_td = pts_j - (td - td_j) * velocity_j;
    // 转到相机系
    Eigen::Vector3d pts_camera_i = pts_i_td / inv_dep_i;  // t_ci_p
    // 转到IMU系
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;  // t_i_p
    // 转到世界系
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;  // t_w_p
    // 转到j的IMU系
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);  // t_ji_p
    // 转到j的相机系
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);  // t_jc_p
    Eigen::Map<Eigen::Vector2d> residual(residuals);

#ifdef UNIT_SPHERE_ERROR
    residual =
        tangent_base * (pts_camera_j.normalized() - pts_j_td.normalized());
#else
    // 计算归一化相机平面上的两点误差
    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>();
#endif

    residual = sqrt_info * residual;

    // gxt:需要注意我们只有三个顶点，有两个没有用到
    if (jacobians) {
      Eigen::Matrix3d Ri = Qi.toRotationMatrix();
      Eigen::Matrix3d Rj = Qj.toRotationMatrix();
      Eigen::Matrix3d ric = qic.toRotationMatrix();
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

      // 下面是计算残差对优化变量的Jacobian
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_pose_i(jacobians[0]);

        Eigen::Matrix<double, 3, 6> jaco_i;
        jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
        jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri *
                                -Utility::skewSymmetric(pts_imu_i);

        jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
        jacobian_pose_i.rightCols<1>().setZero();

        jacobians_.at(0) = jacobian_pose_i.leftCols<6>();
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_pose_j(jacobians[1]);

        Eigen::Matrix<double, 3, 6> jaco_j;
        jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
        jaco_j.rightCols<3>() =
            ric.transpose() * Utility::skewSymmetric(pts_imu_j);

        jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
        jacobian_pose_j.rightCols<1>().setZero();

        jacobians_.at(1) = jacobian_pose_j.leftCols<6>();
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>
            jacobian_ex_pose(jacobians[2]);
        Eigen::Matrix<double, 3, 6> jaco_ex;
        jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri -
                                                   Eigen::Matrix3d::Identity());
        Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
        jaco_ex.rightCols<3>() =
            -tmp_r * Utility::skewSymmetric(pts_camera_i) +
            Utility::skewSymmetric(tmp_r * pts_camera_i) +
            Utility::skewSymmetric(
                ric.transpose() *
                (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
        jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
        jacobian_ex_pose.rightCols<1>().setZero();
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
        jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri *
                           ric * pts_i_td * -1.0 / (inv_dep_i * inv_dep_i);

        jacobians_.at(2) = jacobian_feature;
      }
      if (jacobians[4]) {
        Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
        jacobian_td = reduce * ric.transpose() * Rj.transpose() * Ri * ric *
                          velocity_i / inv_dep_i * -1.0 +
                      sqrt_info * velocity_j.head(2);
      }
    }
  };

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
