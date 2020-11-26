#pragma once

#include "GridState.h"

namespace HybridAStar {
namespace Common {
  // possible directions
  const int GridState::dir = 8;
  // possible movements
  const int GridState::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
  const int GridState::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

  GridState::GridState():GridState(0,0,0,0,nullptr,1) {}

  GridState::GridState(float cellsize,float):GridState(0,0,0,0,nullptr,cellsize) {}

  GridState::GridState(int x, int y, float g, float h, std::shared_ptr<GridState> pred,float cellsize):
    x_(x),y_(y),g_(g),h_(h),pred_(pred),cellSize_(cellsize) {
      this->o_ = false;
      this->c_ = false;
      this->idx_ = -1;
  }
  
  GridState::GridState(const GridState &cp):
    x_(cp.x_),y_(cp.y_),g_(cp.g_),h_(cp.h_),idx_(cp.idx_),o_(cp.o_), c_(cp.c_),pred_(cp.pred_) {}

  GridState &GridState::operator=(const GridState &rhs) {
    x_ = rhs.x_;
    y_ = rhs.y_;
    g_ = rhs.g_;
    h_ = rhs.h_;
    pred_ = rhs.pred_;
    o_ = rhs.o_;
    c_ = rhs.c_;
    idx_ = rhs.idx_;
    return *this;
  }
  
  void GridState::freeState(State *state) const {
    auto *rstate = static_cast<GridState *>(state);
    delete rstate;
  }

  bool GridState::isOnGrid(const int width, const int Height) const {
    return x_ >= 0 && x_ < width && y_ >= 0 && y_ < Height;
  }

  GridState* GridState::createSuccessor(const int i, std::shared_ptr<GridState>& self) {
    int xSucc = x_ + GridState::dx[i];
    int ySucc = y_ + GridState::dy[i];
    return new GridState(xSucc, ySucc, g_, 0, self, cellSize_);
  }

  void GridState::clear() {
    x_ = y_ = g_ = 0;
    pred_.reset(); idx_ = -1;
    o_ = c_ = false;
  }

  bool GridState::operator==(const GridState& rhs) const {
    return (x_ == (rhs).x_) && (y_ == (rhs).y_);
  }
} // namespace Common
} // namespace HybridAStar