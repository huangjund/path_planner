#pragma once

#include <ompl/util/ClassForward.h>

#include "State.h"

#include <iostream>
#include <memory>
#include <cmath>

namespace HybridAStar {
namespace Common {
  /// Grid State is a Rn discrete coordinate state class
  class GridState : State{
  private:
    int x_;
    int y_;
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
    std::shared_ptr<GridState> pred_;
    float cellSize_;
    // CONSTANT VALUES
    // TODO: need to fit high dimension
    /// Number of possible directions
    static const int dir;
    /// Possible movements in the x direction
    static const int dx[];
    /// Possible movements in the y direction
    static const int dy[];
  public:
    // constructor
    explicit GridState();
    explicit GridState(float,float);
    explicit GridState(int x, int y, float g, float h, std::shared_ptr<GridState> pred, float cellsize); 
    
    GridState(const GridState&);
    GridState &operator=(const GridState&);

    ~GridState() = default;

    void freeState (State *state) const;
    /// get the x position
    int getX() const { return x_; }
    /// get the y position
    int getY() const { return y_; }
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
    std::shared_ptr<GridState> getPred() const { return pred_; }
    static const unsigned int dimension = 2;
    unsigned int getDimensions() const {return dimension;}

    /// set the x position
    void setX(const int& x) { x_ = x; }
    /// set the y position
    void setY(const int& y) { y_ = y; }
    /// set the cost-so-far (real value)
    void setG(const float& g_) { this->g_ = g_; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h_) { this->h_ = h_; }
    /// set and get the index of the node in the 2D array
    // TODO: need to match high dimensions
    int setIdx(int width) { this->idx_ = y_*width + x_; return idx_;}
    /// open the node
    void open() { o_ = true; c_ = false; }
    /// close the node
    void close() { c_ = true; o_ = false; }
    /// set the node neither open nor closed
    void reset() { c_ = false; o_ = false; }
    /// set a pointer to the predecessor of the node
    void setPred(std::shared_ptr<GridState> pred) { this->pred_ = pred; }
    GridState* createSuccessor(const int i, std::shared_ptr<GridState>&);

    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG() { g_ += movementCost(*pred_); }
    /// Updates the cost-to-go for the node x' to the goal node.
    void updateH(const GridState& goal) { h_ = movementCost(goal); }
    /// The heuristic as well as the cost measure.
    // TODO: need to match high dimension
    float movementCost(const GridState& pred) const { return sqrt((x_ - pred.x_) * (x_ - pred.x_) + (y_ - pred.y_) * (y_ - pred.y_)); }

    /// Validity check to test, whether the node is in the 2D array.
    // TODO: need to fit high dimension
    bool isOnGrid(const int width, const int height) const;

    void clear();

    /// Creates a successor on a eight-connected grid.
    // TODO: need to fit high dimension
    // GridState* createSuccessor(const int i);

    bool operator==(const GridState& rhs) const;
  };
}
}