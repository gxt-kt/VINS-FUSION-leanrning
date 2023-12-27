/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>
#include <map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;

/**
 * 残差块，可以理解为一条边，一个观测
*/
struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function, ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    // 参数块，例如 [当前帧位姿，后一帧位姿，外参，逆深度，Td]
    std::vector<double *> parameter_blocks;
    // 待Marg的参数，比如需要Marg掉当前帧位姿、逆深度，对应在参数块下的索引
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;

    // 对于位姿（平移+旋转），参数表示是7个维度，残差只用6个（丢掉了四元数的实部）
    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};

class MarginalizationInfo
{
  public:
    MarginalizationInfo(){valid = true;};
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void preMarginalize();
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    // 与当前Marg帧相关的所有残差项
    std::vector<ResidualBlockInfo *> factors;
    // 待Marg的变量个数，保留的变量个数
    int m, n;
    // <变量块起始地址, 变量块尺寸>
    std::unordered_map<long, int> parameter_block_size; //global size
    int sum_block_size;
    // <变量块起始地址，参数块一维索引> 注：需要Marg的变量索引在最前面
    std::unordered_map<long, int> parameter_block_idx; //local size
    // <变量块起始地址，变量值>
    std::unordered_map<long, double *> parameter_block_data;

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
    bool valid;

};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};

#include "problem.h"

class GxtMarginalizationFactor : public Edge {
   public:
    GxtMarginalizationFactor(MarginalizationInfo *_marginalization_info)
        : Edge(20, 20, std::vector<std::string>{"EdgeMarginalization"}),
          marginalization_info(
              _marginalization_info) {  // 这里初始化20不重要，反正后面要重写的
      // int cnt = 0;
      // int ii = 0;
      // [变量块尺寸] 保留变量
      // for (auto it : marginalization_info->keep_block_size) {
      //   // mutable_parameter_block_sizes()->push_back(it);
      //   ++ii;
      //   cnt += it;
      // }
      // printf("residual size: %d, %d\n", cnt, n);
      //  先验残差的维度
      // set_num_residuals(marginalization_info->n);
      ReSet(marginalization_info->n,
            marginalization_info->keep_block_size.size());
    }

    /// 返回边的类型信息
    virtual std::string TypeInfo() const override {
      return "EdgeMarginalization";
    }

    /// 计算残差
    virtual void ComputeResidual() override {
      gDebugWarn("margin ComputeResidual begin");
      VecX residuals_tmp;
      residuals_tmp.resize(marginalization_info->n);
      double *residuals = &residuals_tmp(0);

      // double jacobians[14][40];

      // printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(),
      // num_residuals()); for (int i = 0; i <
      // static_cast<int>(keep_block_size.size()); i++)
      //{
      //     //printf("unsigned %x\n", reinterpret_cast<unsigned
      //     long>(parameters[i]));
      //     //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
      // printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
      // printf("residual %x\n", reinterpret_cast<long>(residuals));
      // }
      //  上一次Marg操作之后保留的变量个数
      int n = marginalization_info->n;
      // 上一次Marg删除的变量个数
      int m = marginalization_info->m;
      Eigen::VectorXd dx(n);
      // 遍历 [变量块尺寸] 保留变量
      for (int i = 0;
           i < static_cast<int>(marginalization_info->keep_block_size.size());
           i++) {
        // 变量块尺寸
        int size = marginalization_info->keep_block_size[i];
        // 之前保存的时候没有减去m，这里减去m，从0开始。keep_block_idx是[变量块索引]
        // 保留变量，注：Marg之前的索引
        int idx = marginalization_info->keep_block_idx[i] - m;
        // 变量当前的值
        // Eigen::VectorXd x =
        //     Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x = verticies_.at(i)->Parameters();
        // 上一次Marg操作之后，变量的值
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(
            marginalization_info->keep_block_data[i], size);
        // dx = x - x0
        if (size != 7)
          dx.segment(idx, size) = x - x0;
        else {
          dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
          dx.segment<3>(idx + 3) =
              2.0 *
              Utility::positify(
                  Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                  Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                  .vec();
          if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                 Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                    .w() >= 0)) {
            dx.segment<3>(idx + 3) =
                2.0 *
                -Utility::positify(
                     Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                     Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                     .vec();
          }
        }
      }
      // 更新残差，r = r0 + J0*dx
      Eigen::Map<Eigen::VectorXd> map_tmp(residuals, n);
      map_tmp = marginalization_info->linearized_residuals +
                marginalization_info->linearized_jacobians * dx;

      residual_ = map_tmp;
      gDebugWarn("margin ComputeResidual end");
    }

    /// 计算雅可比
    virtual void ComputeJacobians() override {
      VecX residuals_tmp;
      residuals_tmp.resize(marginalization_info->n);
      double *residuals = &residuals_tmp(0);

      double jacobians[14][1000];

      // gDebugWarn("margin ComputeJacobians begin");
      // printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(),
      // num_residuals()); for (int i = 0; i <
      // static_cast<int>(keep_block_size.size()); i++)
      //{
      //     //printf("unsigned %x\n", reinterpret_cast<unsigned
      //     long>(parameters[i]));
      //     //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
      // printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
      // printf("residual %x\n", reinterpret_cast<long>(residuals));
      // }
      //  上一次Marg操作之后保留的变量个数
      int n = marginalization_info->n;
      // 上一次Marg删除的变量个数
      int m = marginalization_info->m;
      Eigen::VectorXd dx(n);
      // 遍历 [变量块尺寸] 保留变量
      for (int i = 0;
           i < static_cast<int>(marginalization_info->keep_block_size.size());
           i++) {
        // 变量块尺寸
        int size = marginalization_info->keep_block_size[i];
        // 之前保存的时候没有减去m，这里减去m，从0开始。keep_block_idx是[变量块索引]
        // 保留变量，注：Marg之前的索引
        int idx = marginalization_info->keep_block_idx[i] - m;
        // 变量当前的值
        // Eigen::VectorXd x =
        //     Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x = verticies_.at(i)->Parameters();
        // 上一次Marg操作之后，变量的值
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(
            marginalization_info->keep_block_data[i], size);
        // dx = x - x0
        if (size != 7)
          dx.segment(idx, size) = x - x0;
        else {
          dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
          dx.segment<3>(idx + 3) =
              2.0 *
              Utility::positify(
                  Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                  Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                  .vec();
          if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                 Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                    .w() >= 0)) {
            dx.segment<3>(idx + 3) =
                2.0 *
                -Utility::positify(
                     Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                     Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                     .vec();
          }
        }
      }
      // 更新残差，r = r0 + J0*dx
      Eigen::Map<Eigen::VectorXd> map_tmp(residuals, n);
      map_tmp = marginalization_info->linearized_residuals +
                marginalization_info->linearized_jacobians * dx;

      residual_ = map_tmp;

      if (jacobians) {
        for (int i = 0;
             i < static_cast<int>(marginalization_info->keep_block_size.size());
             i++) {
          if (jacobians[i]) {
            int size = marginalization_info->keep_block_size[i],
                local_size = marginalization_info->localSize(size);
            int idx = marginalization_info->keep_block_idx[i] - m;
            Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                     Eigen::RowMajor>>
                jacobian(jacobians[i], n, size);
          // gDebug(n);
          // gDebug(size);
            jacobian.setZero();
            // J = J0 不变
            jacobian.leftCols(local_size) =
                marginalization_info->linearized_jacobians.middleCols(
                    idx, local_size);

            jacobians_.at(i) = jacobian.leftCols(local_size);
          }
        }
      }
      // gDebugWarn("margin ComputeJacobians end");
    }

    MarginalizationInfo *marginalization_info;
};
