#pragma once

#include "GridState.h"

namespace HybridAStar {
namespace Common {
  GridState::GridState():GridState(0,0,0,0,nullptr) {}

  GridState::GridState(int x, int y, float g, float h, const GridStatePtr& pred):
                        RealVectorState<int>(2),g_(g),h_(h),pred_(pred) {
      values_[0] = x;
      values_[1] = y;
      this->o_ = false;
      this->c_ = false;
      this->idx_ = -1;
  }
  
  GridState::GridState(const GridState &cp):
    RealVectorState<int>(2),g_(cp.g_),h_(cp.h_),idx_(cp.idx_),o_(cp.o_), c_(cp.c_),pred_(cp.pred_) {
      values_[0] = cp[0];
      values_[1] = cp[1];
    }

  GridState& GridState::operator=(const GridState &rhs) {
    values_[0] = rhs[0];
    values_[1] = rhs[1];
    g_ = rhs.g_;
    h_ = rhs.h_;
    pred_ = rhs.pred_;
    o_ = rhs.o_;
    c_ = rhs.c_;
    idx_ = rhs.idx_;
    return *this;
  }

  bool GridState::isOnGrid(const int width, const int Height) const {
    return values_[0] >= 0 && values_[0] < width && values_[1] >= 0 && values_[1] < Height;
  }

  GridStatePtr GridState::createSuccessor(const int i, const GridStatePtr& self) {
    int xSucc = values_[0] + aStarMotionPrimitives::dx[i];
    int ySucc = values_[1] + aStarMotionPrimitives::dy[i];
    return std::make_shared<GridState>(new GridState(xSucc, ySucc, g_, 0, self));
  }

  void GridState::clear() {
    values_[0] = values_[1] = g_ = 0;
    pred_.reset(); idx_ = -1;
    o_ = c_ = false;
  }

  bool GridState::operator==(const GridState& rhs) const {
    return (values_[0] == (rhs).values_[0]) && (values_[1] == (rhs).values_[1]);
  }
} // namespace Common
} // namespace HybridAStar