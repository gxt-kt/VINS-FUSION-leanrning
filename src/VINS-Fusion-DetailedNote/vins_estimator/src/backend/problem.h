#pragma once

#include <cmath>

// 主要用在函数SelfAdjointEigenSolver
#include <eigen3/Eigen/Dense>

#include "edge.h"
#include "vertex.h"

// add fpm
#include "fpm.h"

class Problem {
 public:
  /**
   * 问题的类型
   * SLAM问题还是通用的问题
   *
   * 如果是SLAM问题那么pose和landmark是区分开的，Hessian以稀疏方式存储
   * SLAM问题只接受一些特定的Vertex和Edge
   * 如果是通用问题那么hessian是稠密的，除非用户设定某些vertex为marginalized
   */
  enum class ProblemType { SLAM_PROBLEM, GENERIC_PROBLEM };

  using ulong = unsigned long;

  using HashVertex = std::map<unsigned long, std::shared_ptr<Vertex>>;
  using HashEdge = std::unordered_map<unsigned long, std::shared_ptr<Edge>>;
  using HashVertexIdToEdge =
      std::unordered_multimap<unsigned long, std::shared_ptr<Edge>>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  Problem(ProblemType problem_type) : problem_type_(problem_type) {
    // LogoutVectorSize();
    verticies_marg_.clear();
  };

  ~Problem() {}

  bool AddVertex(std::shared_ptr<Vertex> vertex) {
    // 已经存在了就报错
    if (verticies_.find(vertex->Id()) != verticies_.end()) {
      gDebugWarn("AddVertex is exist");
      return false;
    }
    verticies_.insert({vertex->Id(), vertex});
    return true;
  };

  // gxt: not use
  // bool RemoveVertex(std::shared_ptr<Vertex> vertex);

  bool AddEdge(std::shared_ptr<Edge> edge) {
    if (edges_.find(edge->Id()) != edges_.end()) {
      gDebugWarn("AddEdge is exist");
      return false;
    }

    edges_.insert({edge->Id(), edge});

    for (auto& vertex : edge->Verticies()) {
      vertexToEdge_.insert({vertex->Id(), edge});
    }
    return true;
  };

  // gxt: not use
  // bool RemoveEdge(std::shared_ptr<Edge> edge);

  /**
   * 取得在优化中被判断为outlier部分的边，方便前端去除outlier
   * @param outlier_edges
   */
  // gxt: not use
  // void GetOutlierEdges(std::vector<std::shared_ptr<Edge>>& outlier_edges);

  /**
   * 求解此问题
   * @param iterations
   * @return
   */
  bool Solve(int iterations) {
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
          inverse_depth_max = std::max(VectorGetAbsMax(verticie.second->Parameters()), inverse_depth_max);
        }
        if (verticie.second->TypeInfo() == "VertexImu") {
          ++imu_i;
          imu_speed_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 0, 3), imu_speed_max);
          imu_accbias_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 3, 6), imu_accbias_max);
          imu_gyrobias_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 6, 9), imu_gyrobias_max);
        }
        if (verticie.second->TypeInfo() == "VertexPose") {
          ++pose_i;
          pose_transition_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 0, 3), pose_transition_max);
          pose_quaternion_max = std::max(VectorGetAbsMax(verticie.second->Parameters(), 3, 7), pose_quaternion_max);
        }
      }

      int quantize_inverse_depth_il_test = std::max((int)std::ceil(std::log2(inverse_depth_max)),0);

      int quantize_pose_quaternion_il_test = std::max((int)std::ceil(std::log2(pose_quaternion_max)),0);
      int quantize_pose_transition_il_test = std::max((int)std::ceil(std::log2(pose_transition_max)),0);

      int quantize_imu_speed_il_test = std::max((int)std::ceil(std::log2(imu_speed_max)),0);
      int quantize_imu_accbias_il_test = std::max((int)std::ceil(std::log2(imu_accbias_max)),0);
      int quantize_imu_gyrobias_il_test = std::max((int)std::ceil(std::log2(imu_gyrobias_max)),0);

      gDebugWarn() << VAR(pose_i, imu_i, inverse_depth_i);

      gDebugWarn() << VAR(inverse_depth_max, quantize_inverse_depth_il_test);
      gDebugWarn() << VAR(imu_speed_max, quantize_imu_speed_il_test);
      gDebugWarn() << VAR(imu_accbias_max, quantize_imu_accbias_il_test);
      gDebugWarn() << VAR(imu_gyrobias_max, quantize_imu_gyrobias_il_test);
      gDebugWarn() << VAR(pose_quaternion_max, quantize_pose_quaternion_il_test);
      gDebugWarn() << VAR(pose_transition_max, quantize_pose_transition_il_test);

    }

    // 实际开始量化
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
      hessian_max = std::max(MatrixGetAbsMax(Hessian_), hessian_max);
      b_max = std::max(MatrixGetAbsMax(b_), b_max);
    }
    gDebugWarn() << VAR(hessian_max, b_max);

    int hessian_max_il_test = std::max((int)std::ceil(std::log2(hessian_max)), 0);
    int b_max_il_test = std::max((int)std::ceil(std::log2(b_max)), 0);
    gDebugWarn() << VAR(hessian_max, hessian_max_il_test);
    gDebugWarn() << VAR(b_max, b_max_il_test);

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


 private:

  // gxt: 把变量总数给到ordering_generic_
  /// 设置各顶点的ordering_index
  void SetOrdering() {
    // 每次重新计数
    ordering_poses_ = 0; // 位姿的个数
    ordering_generic_ = 0; // 总的待优化变量的个数
    ordering_landmarks_ = 0; // landmark的个数
    // int debug = 0;

    // Note:: verticies_ 是 map 类型的, 顺序是按照 id 号排序的
    // 统计带估计的所有变量的总维度
    gDebugWarn(verticies_.size());
    for (auto vertex : verticies_) {
      ordering_generic_ += vertex.second->LocalDimension();

      // if (IsPoseVertex(vertex.second)) {
      //   debug += vertex.second->LocalDimension();
      // }

      if (problem_type_ ==
          ProblemType::SLAM_PROBLEM)  // 如果是 slam 问题，还要分别统计 pose 和
                                      // landmark 的维数，后面会对他们进行排序
      {
        AddOrderingSLAM(vertex.second);
      } else if(problem_type_ ==
          ProblemType::GENERIC_PROBLEM) {
        vertex.second->SetOrderingId(ordering_generic_-vertex.second->LocalDimension());
      }
      // if (IsPoseVertex(vertex.second)) {
      //   std::cout << vertex.second->Id()
      //             << " order: " << vertex.second->OrderingId() << std::endl;
      // }
    }
    gDebugWarn(ordering_generic_);

    // std::cout << "\n ordered_landmark_vertices_ size : " << idx_landmark_vertices_.size() << std::endl;
    if (problem_type_ == ProblemType::SLAM_PROBLEM) {
      // 这里要把 landmark 的 ordering 加上 其他所有的 的数量，就保持了 landmark
      // 在后,而 pose 在前
      // 这里要对之前进行分类的进行排序，要让landmark的顶点排在最后

      // ulong all_pose_dimension =
      //     ordering_poses_ + ordering_imus_ + ordering_tds_ + ordering_others_;
      // for (auto landmarkVertex : idx_landmark_vertices_) {
      //   landmarkVertex.second->SetOrderingId(
      //       landmarkVertex.second->OrderingId() + all_pose_dimension);
      // }
      ulong dimension_sum = 0;
      for (auto pose : idx_pose_vertices_) {
        pose.second->SetOrderingId(dimension_sum);
        dimension_sum += pose.second->LocalDimension();
      }
      for (auto imu : idx_imu_vertices_) {
        imu.second->SetOrderingId(dimension_sum);
        dimension_sum += imu.second->LocalDimension();
      }
      for (auto td : idx_td_vertices_) {
        td.second->SetOrderingId(dimension_sum);
        dimension_sum += td.second->LocalDimension();
      }
      for (auto other : idx_other_vertices_) {
        other.second->SetOrderingId(dimension_sum);
        dimension_sum += other.second->LocalDimension();
      }
      for (auto landmark : idx_landmark_vertices_) {
        landmark.second->SetOrderingId(dimension_sum);
        dimension_sum += landmark.second->LocalDimension();
      }

      std::cout << "\n pose size : " << idx_pose_vertices_.size() << std::endl;
      std::cout << "\n imu size : " << idx_imu_vertices_.size() << std::endl;
      std::cout << "\n td size : " << idx_td_vertices_.size() << std::endl;
      std::cout << "\n other size : " << idx_other_vertices_.size() << std::endl;
      std::cout << "\n landmark size : " << idx_landmark_vertices_.size() << std::endl;
    }
  }

  // 这个函数就是把顶点先各自分组，统计每个分组都有啥
  /// set ordering for new vertex in slam problem
  void AddOrderingSLAM(std::shared_ptr<Vertex> v) {
    if (IsPoseVertex(v)) {
      v->SetOrderingId(ordering_poses_);
      idx_pose_vertices_.insert( std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
      ordering_poses_ += v->LocalDimension();
    } else if (IsLandmarkVertex(v)) {
      v->SetOrderingId(ordering_landmarks_);
      idx_landmark_vertices_.insert( std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
      ordering_landmarks_ += v->LocalDimension();
    } else if (IsImuVertex(v)) {
      v->SetOrderingId(ordering_imus_);
      idx_imu_vertices_.insert( std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
      ordering_imus_ += v->LocalDimension();
    } else if (IsTdVertex(v)) {
      v->SetOrderingId(ordering_tds_);
      idx_td_vertices_.insert( std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
      ordering_tds_ += v->LocalDimension();
    } else {
      v->SetOrderingId(ordering_others_);
      idx_other_vertices_.insert( std::pair<ulong, std::shared_ptr<Vertex>>(v->Id(), v));
      ordering_others_ += v->LocalDimension();
      gDebugError("不应该有other顶点 在这个问题里");
    }
  }

  /// 构造大H矩阵
  void MakeHessian() {
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
      printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0],
          rho[1], rho[2]);
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

  /// schur求解SBA
  void SchurSBA();

  /// 解线性方程
  void SolveLinearSystem() {
    // gxt:
    // 主要是求解出delta_x_
    // 其中分成普通问题和SLAM问题
    // 其中普通问题直接求解就好了
    // SLAM由于矩阵比较稀疏，可以使用舒尔补的方式求解

    // 非 SLAM 问题直接求解
    if (problem_type_ == ProblemType::GENERIC_PROBLEM) {
     // delta_x_ = Hessian_.inverse() * b_;
     // delta_x_ = Hessian_.denseSchur().solve(b_);
     delta_x_ = Hessian_.ldlt().solve(b_);
//      gDebug(delta_x_);
    } else if (problem_type_ == ProblemType::SLAM_PROBLEM) {
     // delta_x_ = Hessian_.ldlt().solve(b_);
      // return;
      // SLAM 问题采用舒尔补的计算方式

      // step1: schur marginalization --> Hpp, bpp
      // int reserve_size = ordering_poses_;
      int reserve_size = ordering_generic_-ordering_landmarks_;
      int marg_size = ordering_landmarks_;

      MatXX Hmm = Hessian_.block(reserve_size, reserve_size, marg_size, marg_size);
      MatXX Hpm = Hessian_.block(0, reserve_size, reserve_size, marg_size);
      MatXX Hmp = Hessian_.block(reserve_size, 0, marg_size, reserve_size);
      VecX bpp = b_.segment(0, reserve_size);
      VecX bmm = b_.segment(reserve_size, marg_size);

      // Hmm
      // 是对角线矩阵，它的求逆可以直接为对角线块分别求逆，如果是逆深度，对角线块为1维的，则直接为对角线的倒数，这里可以加速
      MatXX Hmm_inv(MatXX::Zero(marg_size, marg_size));
      for (auto landmarkVertex : idx_landmark_vertices_) {
        int idx = landmarkVertex.second->OrderingId() - reserve_size;
        int size = landmarkVertex.second->LocalDimension();
        Hmm_inv.block(idx, idx, size, size) =
            Hmm.block(idx, idx, size, size).inverse();
      }

      MatXX tempH = Hpm * Hmm_inv;
      H_pp_schur_ =
          Hessian_.block(0, 0, reserve_size, reserve_size) - tempH * Hmp;
      b_pp_schur_ = bpp - tempH * bmm;

      // step2: solve Hpp * delta_x = bpp
      VecX delta_x_pp(VecX::Zero(reserve_size));

      // PCG Solver
      // for (ulong i = 0; i < ordering_poses_; ++i) {
      //   H_pp_schur_(i, i) += my_type{currentLambda_};
      // }

      // int n = H_pp_schur_.rows() * 2;  // 迭代次数
      // delta_x_pp = PCGSolver(H_pp_schur_, b_pp_schur_, n);  // 哈哈，小规模问题，搞 pcg 花里胡哨
      
      // 不整PCG了就，直接ldlt求解吧
      delta_x_pp = H_pp_schur_.ldlt().solve(b_pp_schur_);
      delta_x_.head(reserve_size) = delta_x_pp;
      //        std::cout << delta_x_pp.transpose() << std::endl;

      VecX delta_x_ll(marg_size);
      delta_x_ll = Hmm_inv * (bmm - Hmp * delta_x_pp);
      delta_x_.tail(marg_size) = delta_x_ll;
    }
    // delta_x_ = H.ldlt().solve(b_);
  }

  /// 更新状态变量
  void UpdateStates() {
    for (auto vertex : verticies_) {
      unsigned long idx = vertex.second->OrderingId();
      unsigned long dim = vertex.second->LocalDimension();
      VecX delta = delta_x_.segment(idx, dim);

      // 所有的参数 x 叠加一个增量  x_{k+1} = x_{k} + delta_x
      vertex.second->Plus(delta);
    }
  }

  // 有时候 update 后残差会变大，需要退回去，重来
  void RollbackStates() {
    for (const auto& vertex : verticies_) {
      ulong idx = vertex.second->OrderingId();
      ulong dim = vertex.second->LocalDimension();
      VecX delta = delta_x_.segment(idx, dim);

      // 之前的增量加了后使得损失函数增加了，我们应该不要这次迭代结果，所以把之前加上的量减去。
      vertex.second->Plus(-delta);
    }
  }

  /// 计算并更新Prior部分
  void ComputePrior();

  /// 判断一个顶点是否为Pose顶点
  bool IsPoseVertex(std::shared_ptr<Vertex> v) {
    std::string type = v->TypeInfo();
    return type == std::string("VertexPose");
  }

  /// 判断一个顶点是否为landmark顶点
  bool IsLandmarkVertex(std::shared_ptr<Vertex> v) {
    std::string type = v->TypeInfo();
    return type == std::string("VertexPointXYZ") ||
           type == std::string("VertexInverseDepth");
  }

  /// 判断一个顶点是否为imu顶点
  bool IsImuVertex(std::shared_ptr<Vertex> v) {
    std::string type = v->TypeInfo();
    return type == std::string("VertexImu");
  }

  /// 判断一个顶点是否为Td顶点
  bool IsTdVertex(std::shared_ptr<Vertex> v) {
    std::string type = v->TypeInfo();
    return type == std::string("VertexTd");
  }

  /// 在新增顶点后，需要调整几个hessian的大小
  void ResizePoseHessiansWhenAddingPose(std::shared_ptr<Vertex> v);

  /// 检查ordering是否正确
  bool CheckOrdering();

  // void LogoutVectorSize();

  /// 获取某个顶点连接到的边
  std::vector<std::shared_ptr<Edge>> GetConnectedEdges(
      std::shared_ptr<Vertex> vertex);

  /// Levenberg
  /// 计算LM算法的初始Lambda
  void ComputeLambdaInitLM() {
    ni_ = 2.;
    currentLambda_ = -1.;
    currentChi_ = 0.0;

    // 计算出当前的总残差
    for (const auto& edge : edges_) {
      currentChi_ += edge.second->Chi2();
    }

    // 计算先验的参数（如果有先验的话）
    if (err_prior_.rows() > 0) {
      currentChi_ += static_cast<double>(err_prior_.norm());
    }

    // 1. 第一步计算停止迭代条件stopThresholdLM_
    init_Chi_=currentChi_;
    stopThresholdLM_ = 1e-6 * currentChi_;  // 迭代条件为 误差下降 1e-6 倍

    // 取出H矩阵对角线的最大值
    double maxDiagonal = 0.;
    unsigned long size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (unsigned long i = 0; i < size; ++i) {
      maxDiagonal =
          std::max(std::fabs(static_cast<double>(Hessian_(i, i))), maxDiagonal);
    }
    double tau = 1e-5;
    // 2. 根据对角线最大值计算出currentLambda_
    currentLambda_ = tau * maxDiagonal;  // 给到u0的初值
  }

  /// Hessian 对角线加上或者减去  Lambda
  void AddLambdatoHessianLM() {
    unsigned int size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    for (unsigned long i = 0; i < size; ++i) {
      Hessian_(i, i) += my_type{currentLambda_};
    }
  }

  void RemoveLambdaHessianLM() {
    unsigned long size = Hessian_.cols();
    assert(Hessian_.rows() == Hessian_.cols() && "Hessian is not square");
    // TODO::
    // 这里不应该减去一个，数值的反复加减容易造成数值精度出问题？而应该保存叠加lambda前的值，在这里直接赋值
    for (unsigned int i = 0; i < size; ++i) {
      Hessian_(i, i) -= my_type{currentLambda_};
    }
  }

  /// LM 算法中用于判断 Lambda 在上次迭代中是否可以，以及Lambda怎么缩放
  bool IsGoodStepInLM() {
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
  VecX PCGSolver(const MatXX& A, const VecX& b, int maxIter = -1) {
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

  double currentLambda_;
  double currentChi_;
  double stopThresholdLM_;  // LM 迭代退出阈值条件
  double init_Chi_; // 没优化之前的残差大小
  double ni_;               // 控制 Lambda 缩放大小

  ProblemType problem_type_;

  /// 整个信息矩阵
  MatXX Hessian_;
  VecX b_;
  VecX delta_x_;

  /// 先验部分信息
  MatXX H_prior_;
  VecX b_prior_;
  MatXX Jt_prior_inv_;
  VecX err_prior_;

  /// SBA的Pose部分
  MatXX H_pp_schur_;
  VecX b_pp_schur_;
  // Heesian 的 Landmark 和 pose 部分
  MatXX H_pp_;
  VecX b_pp_;
  MatXX H_ll_;
  VecX b_ll_;

  /// all vertices
  HashVertex verticies_;

  /// all edges
  HashEdge edges_;  // std::unordered_map<unsigned long, std::shared_ptr<Edge>>

  /// 由vertex id查询edge
  HashVertexIdToEdge vertexToEdge_;

  /// Ordering related
  unsigned long ordering_poses_ = 0;
  unsigned long ordering_imus_ = 0;
  unsigned long ordering_tds_ = 0;
  unsigned long ordering_others_ = 0;
  unsigned long ordering_landmarks_ = 0;
  unsigned long ordering_generic_ = 0;

  std::map<unsigned long, std::shared_ptr<Vertex>> idx_pose_vertices_;  // 以ordering排序的pose顶点
  std::map<unsigned long, std::shared_ptr<Vertex>> idx_imu_vertices_;  // 以ordering排序的imu顶点
  std::map<unsigned long, std::shared_ptr<Vertex>> idx_td_vertices_;  // 以ordering排序的td顶点
  std::map<unsigned long, std::shared_ptr<Vertex>> idx_other_vertices_;  // 以ordering排序的other顶点
  std::map<unsigned long, std::shared_ptr<Vertex>> idx_landmark_vertices_;  // 以ordering排序的landmark顶点

  HashVertex verticies_marg_;

  bool bDebug = false;
  double t_hessian_cost_{0};
  double t_PCGsolve_cost{0};
};
