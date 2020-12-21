#pragma once

#include <ompl/util/ClassForward.h>

#include "State.h"
#include "RealVectorState.h"
#include "../parameters/parameters.h"

#include <iostream>
#include <memory>
#include <cmath>

namespace HybridAStar {
namespace Common {
  using GridStatePtr = std::shared_ptr<GridState>;
  /// Grid State is a Rn discrete coordinate state class
  class GridState : public RealVectorState<int>{
  private:
    /// the cost-so-far
    float g_;
    /// the cost-to-go
    float h_;
    /// the index of the node in the 2D array
    int idx_;
    /// the open value
    bool o_;
    /// the closed value
    bool c_;
    /// the predecessor pointer
    GridStatePtr pred_;
    
    // CONSTANT VALUES
    // TODO: need to fit high dimension

  public:
    // constructor
    explicit GridState();
    explicit GridState(int x, int y, float g, float h, const GridStatePtr& pred); 
    
    GridState(const GridState&);
    GridState &operator=(const GridState&);

    ~GridState() override = default;

    /// get the cost-so-far (real value)
    float getG() const { return g_; }
    /// get the cost-to-come (heuristic value)
    float getH() const { return h_; }
    /// get the total estimated cost
    float getC() const { return g_ + h_; }
    /// get the index of the node in the 2D array
    int getIdx() const { return idx_; }
    /// determine whether the node is open
    bool  isOpen() const { return o_; }
    /// determine whether the node is closed
    bool  isClosed() const { return c_; }
    /// get a pointer to the predecessor
    GridStatePtr getPred() const { return pred_; }

    /// set the cost-so-far (real value)
    void setG(const float& g_) { this->g_ = g_; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h_) { this->h_ = h_; }
    /// set and get the index of the node in the 2D array
    // TODO: need to match high dimensions
    int setIdx(int width) { this->idx_ = values_[1]*width + values_[0]; return idx_;}
    /// open the node
    void open() { o_ = true; c_ = false; }
    /// close the node
    void close() { c_ = true; o_ = false; }
    /// set the node neither open nor closed
    void reset() { c_ = false; o_ = false; }
    /// set a pointer to the predecessor of the node
    void setPred(const GridStatePtr& pred) { this->pred_ = pred; }
    GridStatePtr createSuccessor(const int i, const GridStatePtr&);

    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG() { g_ += movementCost(*pred_); }
    /// Updates the cost-to-go for the node x' to the goal node.
    void updateH(const GridState& goal) { h_ = movementCost(goal); }
    /// The heuristic as well as the cost measure.
    // TODO: need to match high dimension
    float movementCost(const GridState& pred) const { 
      return sqrt((values_[0] - pred.values_[0]) * 
                  (values_[0] - pred.values_[0]) + 
                  (values_[1] - pred.values_[1]) * 
                  (values_[1] - pred.values_[1]));
    }

    /// Validity check to test, whether the node is in the 2D array.
    // TODO: need to fit high dimension
    bool isOnGrid(const int width, const int height) const;

    void clear();

    bool operator==(const GridState& rhs) const;
  };
}
}