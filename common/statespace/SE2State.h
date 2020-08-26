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
    float g_;
    float h_;
    int prim_;
    float o_;
    float c_;
    float idx_;
    // TODO: change into intelligent pointer
    const SE2State* pred_;
    float rx_;
    float ry_;
    float rt_;
    /// planning map cell size [unit: meters/cell]
    float cellSize_;
    /// planning map angle size [unit: rad/piece]
    float angleSize_;
    /// Number of possible directions
    static const int dir;
    /// Possible movements in the x direction
    static const float dx[];
    /// Possible movements in the y direction
    static const float dy[];
    /// Possible movements regarding heading theta
    static const float dt[];
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
    const float dubinsShotDistance = 100;

  public:
	  /// collision map cell size [unit: meters/cell]
	  static const float collisionMapCellSize;
  public:
    explicit SE2State();
    explicit SE2State(float cellsize, float anglesize);
    explicit SE2State(float x, float y, float t, float g, float h,
                     SE2State* pred, float cellsize, float anglesize, int prim);
    // copy constructor & assignment
    SE2State(const SE2State &);
    SE2State &operator=(const SE2State &);

    ~SE2State();
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
    static const unsigned int dimension = 3;
    unsigned int getDimensions() const {return dimension;}

    /// set the x position
    void setX(const float& x) { x_ = x; setrx(x);}
    /// set the y position
    void setY(const float& y) { y_ = y; setry(y);}
    /// set the heading theta
    void setT(const float& t) { t_ = t; setrt(t);}

    inline void setrx(const float& x) {rx_ = x/cellSize_; }
    inline void setry(const float& y) {ry_ = y/cellSize_; }
    inline void setrt(const float& t) {rt_ = t/angleSize_;}
    void setRelative(const float& x,const float& y,const float& t){setrx(x);setry(y);setrt(t);}

    /// set the cost-so-far (real value)
    void setG(const float& g) { g_ = g; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h) { h_ = h; }
    /// set and get the index of the node in the 3D grid
    int setIdx(int width, int height) { 
      idx_ = (int)(rt_) * width * height + (int)(ry_) * width + (int)(rx_); return idx_;}
    /// open the node
    void open() { o_ = true; c_ = false;}
    /// close the node
    void close() { c_ = true; o_ = false; }
    /// set a pointer to the predecessor of the node
    void setPred(const SE2State* pred) { pred_ = pred; }

    // UPDATE METHODS
    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG();

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
    bool operator == (const SE2State& rhs) const;

    // GRID CHECKING
    /// Validity check to test, whether the node is in the 3D array.
    bool isOnGrid(const int width, const int height) const;

    // TODO: change this function into a more scientific mode
    bool isInRange(const SE2State&) const;

    // SUCCESSOR CREATION
    /// Creates a successor in the continous space.
    SE2State *createSuccessor(const int i);

    void freeState (State *state) const;

    void clear();
  };
}
}