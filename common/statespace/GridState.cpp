#pragma once

#include "GridState.h"

namespace HybridAStar {
namespace Common {
  // possible directions
  const int GridState::dir = 8;
  // possible movements
  const int GridState::dx[] = { -1, -1, 0, 1, 1, 1, 0, -1 };
  const int GridState::dy[] = { 0, 1, 1, 1, 0, -1, -1, -1 };

  GridState::GridState():GridState(nullptr,0,0,nullptr,1) {
    int values[dimension] = {0};
    values_ = values;
  }

  GridState::GridState(float cellsize,float):GridState(nullptr,0,0,nullptr,cellsize) {
    int values[dimension] = {0};
    values_ = values;
  }

  GridState::GridState(const GridState &cp):g_(cp.g_),h_(cp.h_),idx(cp.idx),o(cp.o), c(cp.c),pred_(cp.pred_) {
    int temp[dimension];
    for (size_t i = 0; i<dimension; ++i)
      temp[i] = cp.values_[i];
    values_ = temp;
  }

  GridState::GridState(int *values, float g, float h, GridState* pred,float cellsize):g_(g),h_(h),pred_(pred),cellSize_(cellsize) {
      this->o = false;
      this->c = false;
      this->idx = -1;
      int temp[dimension];
      for (size_t i = 0; i<dimension; ++i)
        temp[i] = values[i];
      values_ = temp;
  }
  
  GridState &GridState::operator=(const GridState &rhs) {
    g_ = rhs.g_;
    h_ = rhs.h_;
    pred_ = rhs.pred_;
    o = rhs.o;
    c = rhs.c;
    idx = rhs.idx;
    for (size_t i = 0; i<dimension; ++i)
      values_[i] = rhs.values_[i];
    return *this;
  }
  // Eu measure
  double GridState::distance(const State *state1, const State *state2) const {
    double dist = 0;
    const int *s1 = static_cast<const GridState*>(state1)->values_;
    const int *s2 = static_cast<const GridState*>(state2)->values_;

    for(size_t i = 0; i < dimension; ++i) {
      double diff = (*s1++) - (*s2++);
      dist += diff*diff;
    }

    return sqrt(dist);
  }

  void GridState::freeState(State *state) const {
    auto *rstate = static_cast<GridState *>(state);
    delete[] rstate->values_;
    delete rstate;
  }

  void GridState::printState(const State *state, std::ostream &out) const {
    out << "Grid " <<dimension<<"D State: [";
    if (state != nullptr){
      const auto *rstate = static_cast<const GridState *>(state);
      for (size_t i = 0; i < dimension; ++i){
        out << rstate->values_[i];
        if(i+1 < dimension)
          out << ' ';
      }
    } 
    else
      out << "NULLPTR" << std::endl;
    out << "]" << std::endl;
  }

  inline bool GridState::isOnGrid(const int width, const int Height) const {
    return values_[0] >= 0 && values_[0] < width && values_[1] >= 0 && values_[1] < Height;
  }

  // GridState* GridState::createSuccessor(const int i) {

  // }

  bool GridState::operator == (const State& rhs) const {
    for (size_t i = 0; i < dimension; ++i) {
      if(values_[i] != static_cast<const GridState&>(rhs).values_[i])
        return false;
    }
    return true;
  }
} // namespace Common
} // namespace HybridAStar