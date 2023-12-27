#pragma once

#include "problem.h"

class GxtInverseDeep : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  GxtInverseDeep() : Vertex(1) {}

  virtual std::string TypeInfo() const { return "VertexInverseDepth"; }
};

class VertexTd : public Vertex {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexTd() : Vertex(1) {}

    std::string TypeInfo() const {
        return "VertexTd";
    }
};
