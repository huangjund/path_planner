#pragma once

#include <ompl/util/ClassForward.h>

#include "State.h"

#include <iostream>
#include <memory>
#include <cmath>

namespace HybridAStar {
namespace Common {
  OMPL_CLASS_FORWARD(GridState);
  /// Grid State is a Rn discrete coordinate state class
  class GridState : State{
  private:
    
    /// the cost-so-far
    float g_;
    /// the cost-to-go
    float h_;
    /// the index of the node in the 2D array
    int idx;
    /// the open value
    bool o;
    /// the closed value
    bool c;
    /// the predecessor pointer
    const GridState* pred_;
    float cellSize_;
  public:
    int *values_; // coordinate value

    GridState(const GridState &);
    GridState &operator=(const GridState &);

    // constructor
    explicit GridState();
    explicit GridState(float,float);
    explicit GridState(int *values, float g, float h, GridState* pred, float cellsize); 
    ~GridState() = default;

    double distance (const State *state1, const State *state2) const;
    void freeState (State *state) const;
    void printState (const State *state, std::ostream &out) const;
    /// get the x position
    int getX() const { return values_[0]; }
    /// get the y position
    int getY() const { return values_[1]; }
    /// get the cost-so-far (real value)
    float getG() const { return g_; }
    /// get the cost-to-come (heuristic value)
    float getH() const { return h_; }
    /// get the total estimated cost
    float getC() const { return g_ + h_; }
    /// get the index of the node in the 2D array
    int getIdx() const { return idx; }
    /// determine whether the node is open
    bool  isOpen() const { return o; }
    /// determine whether the node is closed
    bool  isClosed() const { return c; }
    /// get a pointer to the predecessor
    const GridState* getPred() const { return pred_; }
    static const unsigned int dimension = 2;
    unsigned int getDimensions() const {return dimension;}

    /// set the x position
    void setX(const int& x) { this->values_[0] = x; }
    /// set the y position
    void setY(const int& y) { this->values_[1] = y; }
    /// set the cost-so-far (real value)
    void setG(const float& g_) { this->g_ = g_; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h_) { this->h_ = h_; }
    /// set and get the index of the node in the 2D array
    // TODO: need to match high dimensions
    int setIdx(int width) { this->idx = values_[1]*width + values_[0]; return idx;}
    /// open the node
    void open() { o = true; c = false; }
    /// close the node
    void close() { c = true; o = false; }
    /// set the node neither open nor closed
    void reset() { c = false; o = false; }
    /// set a pointer to the predecessor of the node
    void setPred(GridState* pred) { this->pred_ = pred; }

    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG() { g_ += movementCost(*pred_); }
    /// Updates the cost-to-go for the node x' to the goal node.
    void updateH(const GridState& goal) { h_ = movementCost(goal); }
    /// The heuristic as well as the cost measure.
    // TODO: need to match high dimension
    float movementCost(const GridState& pred) const { return sqrt((values_[0] - pred.values_[0]) * (values_[0] - pred.values_[0]) + (values_[1] - pred.values_[1]) * (values_[1] - pred.values_[1])); }

    /// Validity check to test, whether the node is in the 2D array.
    // TODO: need to fit high dimension
    bool isOnGrid(const int width, const int height) const;

    /// Creates a successor on a eight-connected grid.
    // TODO: need to fit high dimension
    // GridState* createSuccessor(const int i);

    int operator[](unsigned int i) const
    {
        return values_[i];
    }

    int &operator[](unsigned int i)
    {
        return values_[i];
    }
    bool operator == (const State& rhs) const;

    // CONSTANT VALUES
    // TODO: need to fit high dimension
    /// Number of possible directions
    static const int dir;
    /// Possible movements in the x direction
    static const int dx[];
    /// Possible movements in the y direction
    static const int dy[];
  };
}
}