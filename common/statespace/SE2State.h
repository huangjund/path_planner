#pragma once

#include <memory>
#include <cmath>
#include <iostream>

#include "State.h"
#include "../../utils/Helper.h"

namespace HybridAStar {
namespace Common {
  /**
   * @brief SE2 space include x, y and yaw
   * 
   */
  class SE2State : State {
  private:
    float x_;
    float y_;
    float t_;
    float g_; ///< cost to come
    float h_; ///< heuristic
    int prim_;  ///< primitive from its predecessor
    float o_; ///< open flag
    float c_; ///< close flag
    float idx_; ///< index in state space
    std::shared_ptr<SE2State> pred_;
    float rx_;  ///< relative x value: x/resolution 
    float ry_;  ///< relative y value: y/resolution
    float rt_;///< relative t value: t / resolution
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
    SE2State();
    explicit SE2State(float x, float y, float t);
    explicit SE2State(float cellsize, float anglesize);
    explicit SE2State(float x, float y, float t, float g, float h,
                     std::shared_ptr<SE2State> pred, float cellsize, float anglesize, int prim);
    // copy constructor & assignment
    SE2State(const SE2State &);
    SE2State &operator=(const SE2State &);

    ~SE2State() = default;
    /// get the x position
    float getX() const { return x_; }
    /// get the y position
    float getY() const { return y_; }
    /// get the heading theta
    float getT() const { return t_; }

    /**
     * @brief get relative x
     * 
     * @return float 
     */
    float getrx() const {return rx_;}

    /**
     * @brief get relative y
     * 
     * @return float 
     */
    float getry() const {return ry_;}

    /**
     * @brief get relative t 
     * 
     * @return float 
     */
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
    std::shared_ptr<SE2State> getPred() const { return pred_; }
    static const unsigned int dimension = 3;
    unsigned int getDimensions() const {return dimension;}

    /// set the x position
    void setX(const float& x) { x_ = x; setrx(x);}
    /// set the y position
    void setY(const float& y) { y_ = y; setry(y);}
    /// set the heading theta
    void setT(const float& t) { t_ = std::fmod(t,2*M_PI); setrt(t_);}

    /**
     * @brief set relative x
     * 
     * @param x 
     */
    inline void setrx(const float& x) {rx_ = x/cellSize_; }

    /**
     * @brief set relative y
     * 
     * @param y 
     */
    inline void setry(const float& y) {ry_ = y/cellSize_; }

    /**
     * @brief set relative t
     * 
     * @param t 
     */
    inline void setrt(const float& t) {rt_ = std::fmod(t,2*M_PI)/angleSize_;}
    void setRelative(const float& x,const float& y,const float& t){setrx(x);setry(y);setrt(t);}

    /// set the cost-so-far (real value)
    void setG(const float& g) { g_ = g; }
    /// set the cost-to-come (heuristic value)
    void setH(const float& h) { h_ = h; }
    /// set and get the index of the node in the 3D grid
    int setIdx(int width, int height) { 
      idx_ = (int)(rt_) * width * height + (int)(ry_) * width + (int)(rx_); return idx_;}
    
    /// set the primitive. this method should not often be called
    void setPrim(const int prim) {prim_ = prim;}
    /// open the node
    void open() { o_ = true; c_ = false;}
    /// close the node
    void close() { c_ = true; o_ = false; }
    /// set a pointer to the predecessor of the node
    void setPred(std::shared_ptr<SE2State> pred) { pred_ = pred; }

    // UPDATE METHODS
    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG();

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
    bool operator == (const SE2State& rhs) const;

    // GRID CHECKING
    /// Validity check to test, whether the node is in the 3D array.
    bool isOnGrid(const int width, const int height) const;

    /**
     * @brief if the state is in map
     * 
     * @return true 
     * @return false 
     */
    bool isInRange(const SE2State&) const;

    // SUCCESSOR CREATION
    /// Creates a successor in the continous space.
    SE2State *createSuccessor(const int i, std::shared_ptr<SE2State>&);

    void freeState (State *state) const;

    void clear();
  };
}
}