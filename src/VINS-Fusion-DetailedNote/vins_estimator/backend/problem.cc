#include "problem.h"



// double inverse_depth_max=0;
// double imu_speed_max=0;
// double imu_accbias_max=0;
// double imu_gyrobias_max=0;
// double pose_transition_max=0;
// double pose_quaternion_max=0;
// double hessian_max=0;
// double b_max=0;

/**
 * 求解此问题
 * @param iterations
 * @return
 */
bool Problem::Solve(int iterations) {
  if (edges_.size() == 0 || verticies_.size() == 0) {
    gDebugWarn() << "\nCannot solve problem without edges or verticies";
    return false;
  }

  // // fpm:
  // // 设置好最大值和整数位宽
  if (problem_type_ == ProblemType::SLAM_PROBLEM) {
    int inverse_depth_i = 0;
    int imu_i = 0;
    int pose_i = 0;
    for (const auto& verticie : verticies_) {
      if (verticie.second->TypeInfo() == "VertexInverseDepth") {
        ++inverse_depth_i;
        // inverse_depth_max = std::max(VectorGetAbsMax(verticie.second->Parameters()), inverse_depth_max);
        const auto& inverse_depth = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(inverse_depth);
      }
      if (verticie.second->TypeInfo() == "VertexImu") {
        ++imu_i;
        // imu_speed_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 0, 3), imu_speed_max);
        // imu_accbias_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 3, 6), imu_accbias_max);
        // imu_gyrobias_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 6, 9), imu_gyrobias_max);
        const auto& imu_speed = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(imu_speed,0,3);
        const auto& imu_accbias = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(imu_accbias,3,6);
        const auto& imu_gyrobias = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(imu_gyrobias,6,9);
      }
      if (verticie.second->TypeInfo() == "VertexPose") {
        ++pose_i;
        // pose_transition_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 0, 3), pose_transition_max);
        // pose_quaternion_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 3, 7), pose_quaternion_max);
        const auto& pose_transition = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(pose_transition,0,3);
        const auto& pose_quaternion = verticie.second->Parameters();
        QUANTIZE_PRINT_MAX(pose_quaternion,3,7);
      }
    }

    // int quantize_inverse_depth_il_test = std::max((int)std::ceil(std::log2(inverse_depth_max)),0);
    //
    // int quantize_pose_quaternion_il_test = std::max((int)std::ceil(std::log2(pose_quaternion_max)),0);
    // int quantize_pose_transition_il_test = std::max((int)std::ceil(std::log2(pose_transition_max)),0);
    //
    // int quantize_imu_speed_il_test = std::max((int)std::ceil(std::log2(imu_speed_max)),0);
    // int quantize_imu_accbias_il_test = std::max((int)std::ceil(std::log2(imu_accbias_max)),0);
    // int quantize_imu_gyrobias_il_test = std::max((int)std::ceil(std::log2(imu_gyrobias_max)),0);

    gDebugWarn() << VAR(pose_i, imu_i, inverse_depth_i);

    // gDebugWarn() << VAR(inverse_depth_max, quantize_inverse_depth_il_test);
    // gDebugWarn() << VAR(imu_speed_max, quantize_imu_speed_il_test);
    // gDebugWarn() << VAR(imu_accbias_max, quantize_imu_accbias_il_test);
    // gDebugWarn() << VAR(imu_gyrobias_max, quantize_imu_gyrobias_il_test);
    // gDebugWarn() << VAR(pose_quaternion_max, quantize_pose_quaternion_il_test);
    // gDebugWarn() << VAR(pose_transition_max, quantize_pose_transition_il_test);

  }

#define GET_QUANTIZE_PARAMETER(para)                         \
static auto para = []() {                                  \
  auto para##_ = yaml_config["quantize"][#para].as<int>(); \
  gDebugWarn(para##_);                 \
  return para##_;                                          \
}();

  GET_QUANTIZE_PARAMETER(quantize_pose_transition_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_pose_transition_il);
  GET_QUANTIZE_PARAMETER(quantize_pose_quaternion_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_pose_quaternion_il);
  GET_QUANTIZE_PARAMETER(quantize_imu_speed_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_imu_speed_il);
  GET_QUANTIZE_PARAMETER(quantize_imu_accbias_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_imu_accbias_il);
  GET_QUANTIZE_PARAMETER(quantize_imu_gyrobias_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_imu_gyrobias_il);
  GET_QUANTIZE_PARAMETER(quantize_inverse_depth_bit_width);
  GET_QUANTIZE_PARAMETER(quantize_inverse_depth_il);

  // std::terminate();

  // 实际开始量化
  static bool enable_quantize = []() {
    bool enable_quantize_ = yaml_config["flag"]["enable_quantize"].as<bool>();
    gDebugWarn(enable_quantize_);
    return enable_quantize_;
  }();
  if(enable_quantize) {
    gDebugWarn() << "enable_quantize";
    if (problem_type_ == ProblemType::SLAM_PROBLEM) {
      for (const auto& verticie : verticies_) {
        if (verticie.second->TypeInfo() == "VertexPose") {
          // gDebugWarn() << G_PRINT_CNT(1) << "Before cut" << verticie.second->Parameters();
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_pose_transition_bit_width,quantize_pose_transition_bit_width-quantize_pose_transition_il,0,3);
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_pose_quaternion_bit_width,quantize_pose_quaternion_bit_width-quantize_pose_quaternion_il,3,7);
          // gDebugWarn() << G_PRINT_CNT(1) << "After cut"<< verticie.second->Parameters();
        }
        if (verticie.second->TypeInfo() == "VertexImu") {
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_imu_speed_bit_width,quantize_imu_speed_bit_width-quantize_imu_speed_il,0,3);
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_imu_accbias_bit_width,quantize_imu_accbias_bit_width-quantize_imu_accbias_il,3,6);
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_imu_gyrobias_bit_width,quantize_imu_gyrobias_bit_width-quantize_imu_gyrobias_il,6,9);
        }
        if (verticie.second->TypeInfo() == "VertexInverseDepth") {
          QuantizeParamCutOff(verticie.second->Parameters(),quantize_inverse_depth_bit_width,quantize_inverse_depth_bit_width-quantize_inverse_depth_il);
        }
      }
    }
  }

  // 统计优化变量的维数，为构建 H 矩阵做准备
  SetOrdering();
  // 遍历edge, 构建 H = J^T * J 矩阵
  MakeHessian();

  if (problem_type_ == ProblemType::SLAM_PROBLEM) {
    // hessian_max = std::max(MatrixGetAbsMax(Hessian_), hessian_max);
    // b_max = std::max(MatrixGetAbsMax(b_), b_max);
    QUANTIZE_PRINT_MAX(Hessian_);
    QUANTIZE_PRINT_MAX(b_);
  }

  // int hessian_max_il_test = std::max((int)std::ceil(std::log2(hessian_max)), 0);
  // int b_max_il_test = std::max((int)std::ceil(std::log2(b_max)), 0);
  // gDebugWarn() << VAR(hessian_max, hessian_max_il_test);
  // gDebugWarn() << VAR(b_max, b_max_il_test);

  // LM 初始化
  ComputeLambdaInitLM();

  bool stop = false;
  int iter = 0;

  while (!stop && iter < iterations) {
    // gDebugCol1() << "iter: " << iter << " , chi= " << currentChi_
    //              << " , Lambda= " << currentLambda_;
    gDebugCol1() << VAR(iter,init_Chi_,currentChi_,currentLambda_);
    bool one_step_success{false};
    int false_cnt = 0;
    while (!one_step_success) {  // 不断尝试 Lambda, 直到成功迭代一步
      // 更新Hx=b为(H+uI)x=b也就是H变为H+uI
      AddLambdatoHessianLM();

      // 解线性方程 H =b x=H(-1)b
      SolveLinearSystem();

      // 把H+uI恢复到原来的H
      RemoveLambdaHessianLM();

      // 优化退出条件1： delta_x_ 很小则退出
      gDebug(delta_x_.squaredNorm());
      if (delta_x_.squaredNorm() <= my_type{1e-8} || false_cnt > 10) {
        gDebug("stop=true : delta_x_.squaredNorm() <= 1e-6 || false_cnt > 10");
        stop = true;
        break;
      }

      // 更新状态量 X = X+ delta_x
      UpdateStates();

      // 判断当前步是否可行以及 LM 的 lambda 怎么更新
      one_step_success = IsGoodStepInLM();
      gDebugCol2() << VAR(one_step_success,false_cnt);

      // 后续处理，
      if (one_step_success) {
        // 在新线性化点 构建 hessian
        MakeHessian();
        // TODO:: 这个判断条件可以丢掉，条件 b_max <= 1e-12
        // 很难达到，这里的阈值条件不应该用绝对值，而是相对值
        //                double b_max = 0.0;
        //                for (int i = 0; i < b_.size(); ++i) {
        //                    b_max = max(fabs(b_(i)), b_max);
        //                }
        //                // 优化退出条件2： 如果残差 b_max
        //                已经很小了，那就退出 stop = (b_max <= 1e-12);
        false_cnt = 0;
      } else {
        false_cnt++;
        RollbackStates();  // 误差没下降，回滚
      }
    }
    iter++;

    if (std::sqrt(currentChi_) <= stopThresholdLM_) {
      stop = true;
    }
  }

  return true;
}


/// 构造大H矩阵
void Problem::MakeHessian() {
  // 代优化变量总数
  unsigned long size = ordering_generic_;

  MatXX H(MatXX::Zero(size, size));
  VecX b(VecX::Zero(size));

  // 遍历每个残差，并计算他们的雅克比，得到最后的 H = J^T * J
  // gDebugWarn("caculate edge begin");
  for (auto& edge : edges_) {
    // gDebugWarn("caculate residual and jacobians begin");
    edge.second->ComputeResidual();
    edge.second->ComputeJacobians();
    // edge.second->CalculateLossFunction();
    // gxt::ScaleResiduals(edge.second);
    double sq_norm = edge.second->residual_.squaredNorm();
 // auto loss_function = std::unique_ptr<gxt::LossFunction>(new gxt::HuberLoss(1.0));
    double rho[3]={-1,-1,-1};
    if(edge.second->loss_function_) {
      edge.second->loss_function_->Evaluate(sq_norm, rho);
    }
    // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0],
    //     rho[1], rho[2]);
    // gDebugWarn("caculate residual and jacobians end");

    std::vector<MatXX> jacobians = edge.second->Jacobians();
    std::vector<std::shared_ptr<Vertex>> verticies = edge.second->Verticies();
    assert(jacobians.size() == verticies.size());

    for (size_t i = 0; i < verticies.size(); ++i) {
      auto v_i = verticies.at(i);
      if (v_i->IsFixed()) {
        continue;  // Hessian 里不需要添加它的信息，也就是它的雅克比为 0
      }

      MatXX jacobian_i = jacobians.at(i);
      unsigned long index_i = v_i->OrderingId();
      unsigned long dim_i = v_i->LocalDimension();

      MatXX JtW = jacobian_i.transpose() * edge.second->Information();

      for (size_t j = i; j < verticies.size(); ++j) {
        auto v_j = verticies.at(j);
        if (v_j->IsFixed()) {
          continue;
        }

        MatXX jacobian_j = jacobians[j];
        unsigned long index_j = v_j->OrderingId();
        unsigned long dim_j = v_j->LocalDimension();
        assert(v_j->OrderingId() != -1);
        
        MatXX hessian = JtW * jacobian_j;
        
        if(edge.second->loss_function_) {
          Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(
              edge.second->residual_.rows(), edge.second->residual_.rows());
          // hessian=JtW * (rho[1]*identity+2*rho[2]*edge.second->residual_*edge.second->residual_.transpose())*jacobian_j;
          hessian=JtW * (rho[1]*identity)*jacobian_j;
        }
        
        // 所有的信息矩阵叠加起来
        H.block(index_i, index_j, dim_i, dim_j).noalias() += hessian;
        if (j != i) {
          // 对称的下三角
          H.block(index_j, index_i, dim_j, dim_i).noalias() +=
              hessian.transpose();
        }
      }
      if(edge.second->loss_function_) {
        b.segment(index_i, dim_i).noalias() -= rho[1]*JtW * edge.second->Residual();
      } else {
        b.segment(index_i, dim_i).noalias() -= JtW * edge.second->Residual();
      }
    }
  }
  // gDebugWarn("caculate edge end");
  Hessian_ = H;
  b_ = b;
  // t_hessian_cost_;// gxt:时间貌似不重要在这里

  // gDebug(H);
  // gDebug(Hessian_);
  // gDebug(b);
  // gDebug(b_);

  delta_x_ = VecX::Zero(size);  // initial delta_x = 0_n;
}


/// LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
bool Problem::IsGoodStepInLM() {
  double scale = 0;
  // scale = delta_x_.transpose() * (my_type{currentLambda_} * delta_x_ + b_);
  scale =
      static_cast<double>((delta_x_.transpose() *
                           (my_type{currentLambda_} * delta_x_ + b_))(0, 0));
  // my_type scale_tmp = delta_x_.transpose() * (my_type{currentLambda_} *
  // delta_x_ + b_);
  // gDebugCol3(delta_x_);
  // gDebugCol3(currentLambda_);
  // gDebugCol3(b_);
  // gDebugCol3(scale);
  // gDebugCol3(scale_tmp);
  // gDebugCol4() << G_SPLIT_LINE;
  // gDebugCol4(my_type{currentLambda_} * delta_x_ + b_);
  // gDebugCol4(delta_x_.transpose());
  // gDebugCol4(delta_x_.transpose() *
  //            (my_type{currentLambda_} * delta_x_ + b_));
  scale += 1e-3;  // make sure it's non-zero :)

  // recompute residuals after update state
  // 统计所有的残差
  double tempChi = 0.0;
  for (auto edge : edges_) {
    edge.second->ComputeResidual();
    tempChi += edge.second->Chi2();
  }

  // gDebugCol5(tempChi);
  // gDebugCol5(currentChi_);

  double rho = (currentChi_ - tempChi) / scale;
  // gDebugCol5(rho);

  // std::terminate();

  if (rho > 0 && std::isfinite(tempChi)) {  // last step was good, 误差在下降
    double alpha = 1. - pow((2 * rho - 1), 3);
    alpha = std::min(alpha, 2.0 / 3.0);
    double scaleFactor = std::max(1.0 / 3.0, alpha);
    currentLambda_ *= scaleFactor;
    ;
    ni_ = 2;
    currentChi_ = tempChi;
    return true;
  } else {
    currentLambda_ *= ni_;
    ni_ *= 2;
    return false;
  }
}

/// PCG 迭代线性求解器
/// 用在SLAM舒尔补求解中
VecX Problem::PCGSolver(const MatXX& A, const VecX& b, int maxIter) {
  assert(A.rows() == A.cols() &&
         "PCG solver ERROR: A is not a square matrix");
  int rows = b.rows();
  int n = maxIter < 0 ? rows : maxIter;
  VecX x(VecX::Zero(rows));
  MatXX M_inv = A.diagonal().asDiagonal().inverse();
  VecX r0(b);  // initial r = b - A*0 = b
  VecX z0 = M_inv * r0;
  VecX p(z0);
  VecX w = A * p;
  double r0z0 = static_cast<double>(r0.dot(z0));
  double alpha = r0z0 / static_cast<double>(p.dot(w));
  VecX r1 = r0 - my_type{alpha} * w;
  int i = 0;
  double threshold = 1e-6 * static_cast<double>(r0.norm());
//    while (static_cast<double>(r1.norm()) > threshold && i < n) {
  while (true) {
    // NOTE: gxt: 注意这里有个大坑，直接用my_type的norm()会触发assert因为数字变小了？不懂，哈哈
    my_type r1_norm=my_type{r1.cast<double>().norm()};
    double r1_norm_double=static_cast<double>(r1_norm);
    if(r1_norm_double<=threshold) {break;}
    if(i>=n) {break;}

    i++;
    VecX z1 = M_inv * r1;
    double r1z1 = static_cast<double>(r1.dot(z1));
    double belta = r1z1 / r0z0;
    z0 = z1;
    r0z0 = r1z1;
    r0 = r1;
    p = my_type{belta} * p + z1;
    w = A * p;
    alpha = r1z1 / static_cast<double>(p.dot(w));
    x += my_type{alpha} * p;
    r1 -= my_type{alpha} * w;
  }
  return x;
}
