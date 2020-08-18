#pragma once

#include <memory>
#include <cmath>
#include <iostream>

#include "State.h"
#include "../../utils/Helper.h"

namespace HybridAStar {
namespace Common {
  // the SE2State class contains discretization information
  class SE2State : State {
  private:
    float x_;
    float y_;
    float t_;
    float rx_;
    float ry_;
    float rt_;
    float g_;
    float h_;
    float idx_;
    float o_;
    float c_;
    int prim_;
    // TODO: change into intelligent pointer
    const SE2State* pred_;
    /// Number of possible directions
    static const int dir;
    /// Possible movements in the x direction
    static const float dx[];
    /// Possible movements in the y direction
    static const float dy[];
    /// Possible movements regarding heading theta
    static const float dt[];
    /// collision map cell size [unit: meters/cell]
    static const float collisionMapCellSize;
    /// planning map cell size [unit: meters/cell]
    const float cellSize_ = 0.5;
    /// planning map angle size [unit: rad/piece]
    const float angleSize_;
    /// TODO: some temporal values
    ///  some should be clustered to Motion Primitives class
    /// some should be clustered to Map
    const float penaltyTurning = 1.05;
    const float penaltyReversing = 2.0;
    const float penaltyCOD = 2.0;
    const int headings = 72;
    const float deltaHeadingRad = 2*M_PI/(float)headings;
    const float deltaHeadingDeg = 360/(float)headings;
    const float deltaHeadingNegRad = 2*M_PI - deltaHeadingRad;

  public:
    SE2State() = default;
    SE2State(float cellsize, float anglesize):SE2State(0,0,0,0,0,nullptr,cellsize,anglesize){}
    explicit SE2State(float x, float y, float t, float g, float h,
                     SE2State* pred, float cellsize, float anglesize, int prim = 0):
                    pred_(pred), cellSize_(cellsize), angleSize_(anglesize){
      x_ = x;
      y_ = y;
      t_ = t;
      g_ = g;
      h_ = h;
      o_ = false;
      c_ = false;
      idx_ = -1;
      prim_ = prim;
      rx_ = x/cellSize_;
      ry_ = y/cellSize_;
      rt_ = t/angleSize_;
    }
    // no copy no assign
    SE2State(const SE2State &) = delete;
    SE2State &operator=(const SE2State &) = delete;

    ~SE2State() = default;
    /// get the x position
    float getX() const { return x_; }
    /// get the y position
    float getY() const { return y_; }
    /// get the heading theta
    float getT() const { return t_; }

    float getrx() const {return rx_;}
    float getry() const {return ry_;}
    float getrt() const {return rt_;}

    /// get the cost-so-far (real value)
    float getG() const { return g_; }
    /// get the cost-to-come (heuristic value)
    float getH() const { return h_; }
    /// get the total estimated cost
    float getC() const { return g_ + h_; }
    /// get the index of the node in the 3D array
    int getIdx() const { return idx_; }
    /// get the number associated with the motion primitive of the node
    int getPrim() const { return prim_; }
    /// determine whether the node is open
    bool isOpen() const { return o_; }
    /// determine whether the node is closed
    bool isClosed() const { return c_; }
    /// determine whether the node is open
    const SE2State *getPred() const { return pred_; }

    /// set the x position
    void setX(const float& x) { x_ = x; setrx(x);}
    /// set the y position
    void setY(const float& y) { y_ = y; setry(y);}
    /// set the heading theta
    void setT(const float& t) { t_ = t; setrt(t);}

    void setrx(const float& x) {rx_ = x/cellSize_; }
    void setry(const float& y) {ry_ = y/cellSize_; }
    void setrt(const float& t) {rt_ = t/angleSize_;}

    /// set the cost-so-far (real value)
    void setG(const float& g) { g_ = g; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h) { h_ = h; }
    /// set and get the index of the node in the 3D grid
    int setIdx(int width, int height) { idx_ = (int)(rt_) * width * height + (int)(ry_) * width + (int)(rx_); return idx_;}
    /// open the node
    void open() { o_ = true; c_ = false;}
    /// close the node
    void close() { c_ = true; o_ = false; }
    /// set a pointer to the predecessor of the node
    void setPred(SE2State* pred) { pred_ = pred; }

    // UPDATE METHODS
    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG();

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
    bool operator == (const SE2State& rhs) const;

    // GRID CHECKING
    /// Validity check to test, whether the node is in the 3D array.
    bool isOnGrid(const int width, const int height) const;

    // SUCCESSOR CREATION
    /// Creates a successor in the continous space.
    SE2State *createSuccessor(const int i);

    void freeState (State *state) const;
  };
}
}