#ifndef _HYBRID_A_STAR_SE2_HYBRID_A_STAR_STATE_H_
#define _HYBRID_A_STAR_SE2_HYBRID_A_STAR_STATE_H_

#include <cmath>
#include <memory>

#include "SE2State.h"
#include "../parameters/parameters.h"
#include "../../utils/Helper.h"

namespace HybridAStar {
namespace Common {
using SE2HAStarPtr = std::shared_ptr<SE2HAStarState>;
class SE2HAStarState final: public SE2State {
  private:
    float g_;
    float h_;
    int prim_;
    float o_;
    float c_;
    float idx_;
    
    SE2HAStarPtr pred_;
    float rx_;
    float ry_;
    float rt_;

    const float penaltyTurning = 1.05;
    const float penaltyReversing = 2.0;
    const float penaltyCOD = 2.0;
    
    const float dubinsShotDistance = 100;

  public:
    SE2HAStarState();
    explicit SE2HAStarState(float x, float y, float t);
    explicit SE2HAStarState(float x, float y, float t, float g, float h,
                            const SE2HAStarPtr& pred, int prim);
    // copy constructor & assignment
    SE2HAStarState(const SE2HAStarState&);
    SE2HAStarState &operator=(const SE2HAStarState&);

    ~SE2HAStarState() override = default;

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
    SE2HAStarPtr getPred() const { return pred_; }

    /// set the x position
    void setX(const float& x) { getMutableX() = x; setrx(x);}
    /// set the y position
    void setY(const float& y) { getMutableY() = y; setry(y);}
    /// set the heading theta
    void setT(const float& t) { getMutableYaw() = t; setrt(t);}

    inline void setrx(const float& x) {rx_ = x/PlanningMapConst::cellSize; }
    inline void setry(const float& y) {ry_ = y/PlanningMapConst::cellSize; }
    inline void setrt(const float& t) {rt_ = t/PlanningMapConst::angleSize;}
    void setRelatives(const float& x,const float& y,const float& t){setrx(x);setry(y);setrt(t);}

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
    void setPred(SE2HAStarPtr pred) { pred_ = pred; }

    // UPDATE METHODS
    /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
    void updateG();

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
    bool operator==(const SE2HAStarState& rhs) const;

    // GRID CHECKING
    /// Validity check to test, whether the node is in the 3D array.
    bool isOnGrid(const int width, const int height) const;

    // TODO: change this function into a more scientific mode
    bool isInRange(const SE2HAStarState&) const;

    // SUCCESSOR CREATION
    /// Creates a successor in the continous space.
    SE2HAStarPtr createSuccessor(const int i, const SE2HAStarPtr&);

    void clear();
};
} // namespace Common
} // namespace HybridAStar
#endif