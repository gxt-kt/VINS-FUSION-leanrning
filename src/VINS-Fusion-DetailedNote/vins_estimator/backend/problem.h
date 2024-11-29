#pragma once

#include <cmath>

// 主要用在函数SelfAdjointEigenSolver
#include <eigen3/Eigen/Dense>

#include "edge.h"
#include "vertex.h"

// add fpm
#include "fpm.h"
#include "yaml_config.h"

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
  bool Solve(int iterations);

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
  void MakeHessian();

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
  bool IsGoodStepInLM();

  /// PCG 迭代线性求解器
  /// 用在SLAM舒尔补求解中
  VecX PCGSolver(const MatXX& A, const VecX& b, int maxIter = -1);

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
