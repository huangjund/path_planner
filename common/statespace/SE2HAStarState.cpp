#include "SE2HAStarState.h"

namespace HybridAStar {
namespace Common {
  SE2HAStarState::SE2HAStarState():SE2HAStarState(0,0,0,0,0,nullptr,0){}

  SE2HAStarState::SE2HAStarState(float x, float y, float t):SE2HAStarState(x,y,t,0,0,nullptr,0) {}

  SE2HAStarState::SE2HAStarState(float x, float y, float t, float g, float h,
                                 const SE2HAStarPtr& pred, int prim = 0):
                                g_(g),h_(h),prim_(prim),o_(false),c_(false),
                                idx_(-1), pred_(pred){
    setX(x); setY(y); setT(t);
  }

  SE2HAStarState::SE2HAStarState(const SE2HAStarState& cp):
                    g_(cp.g_),h_(cp.h_),prim_(cp.prim_),
                    o_(cp.o_),c_(cp.c_),idx_(cp.idx_),pred_(cp.pred_){
    setX(cp.getX());
    setY(cp.getY());
    setT(cp.getYaw());
  }

  SE2HAStarState& SE2HAStarState::operator=(const SE2HAStarState& rhs) {
    setX(rhs.getX());
    setY(rhs.getY());
    setT(rhs.getYaw());
    this->g_ = rhs.g_; 
    this->h_ = rhs.h_; this->prim_ = rhs.prim_; 
    this->o_ = rhs.o_; this->c_ = rhs.c_; 
    this->idx_ = rhs.idx_; this->pred_ = rhs.pred_;
    return *this;
  }

  bool SE2HAStarState::isOnGrid(const int width, const int height) const {
    return rx_ >= 0 && ry_ >= 0 && rx_ < width && ry_ < height;
  }

  bool SE2HAStarState::isInRange(const SE2HAStarState& goal) const {
    int random = rand() % 10 + 1;
    float dx = std::abs(getX() - goal.getX()) / random;
    float dy = std::abs(getY() - goal.getY()) / random;
    return (dx * dx) + (dy * dy) < dubinsShotDistance;
  }

  SE2HAStarPtr SE2HAStarState::createSuccessor(const int i, const SE2HAStarPtr& self) {
    float xSucc;
    float ySucc;
    float tSucc;
    double t = getYaw();

    // calculate successor positions forward
    if (i < 3) {
      xSucc = getX() + HaStarMotionPrimitives::dx[i] * cos(t) - HaStarMotionPrimitives::dy[i] * sin(t);
      ySucc = getY() + HaStarMotionPrimitives::dx[i] * sin(t) + HaStarMotionPrimitives::dy[i] * cos(t);
      tSucc = HybridAStar::Utils::normalizeHeadingRad(t + HaStarMotionPrimitives::dt[i]);
    }
    // backwards
    else {
      xSucc = getX() - HaStarMotionPrimitives::dx[i - 3] * cos(t) - HaStarMotionPrimitives::dy[i - 3] * sin(t);
      ySucc = getY() - HaStarMotionPrimitives::dx[i - 3] * sin(t) + HaStarMotionPrimitives::dy[i - 3] * cos(t);
      tSucc = HybridAStar::Utils::normalizeHeadingRad(t - HaStarMotionPrimitives::dt[i - 3]);
    }
    return std::make_shared<SE2HAStarState>(
            new SE2HAStarState(xSucc,ySucc,tSucc,g_,0,self,i));
  }

  void SE2HAStarState::updateG() {
    // forward driving
    if (prim_ < 3) {
      // penalize turning
      if (pred_->prim_ != prim_) {
        // penalize change of direction
        if (pred_->prim_ > 2) {
          g_ += HaStarMotionPrimitives::dx[0] * penaltyTurning * penaltyCOD;
        } else {
          g_ += HaStarMotionPrimitives::dx[0] * penaltyTurning;
        }
      } else {
        g_ += HaStarMotionPrimitives::dx[0];
      }
    }
    // reverse driving
    else {
      // penalize turning and reversing
      if (pred_->prim_ != prim_) {
        // penalize change of direction
        if (pred_->prim_ < 3) {
          g_ += HaStarMotionPrimitives::dx[0] * penaltyTurning * penaltyReversing * penaltyCOD;
        } else {
          g_ += HaStarMotionPrimitives::dx[0] * penaltyTurning * penaltyReversing;
        }
      } else {
        g_ += HaStarMotionPrimitives::dx[0] * penaltyReversing;
      }
    }
  }

  bool SE2HAStarState::operator==(const SE2HAStarState& rhs) const {
    double t = getYaw(), rt = rhs.getYaw();
    return (int)rx_ == (int)rhs.rx_ &&
           (int)ry_ == (int)rhs.ry_ &&
           (std::abs(t - rt) <= HaStarMotionPrimitives::deltaHeadingRad ||
            std::abs(t - rt) >= HaStarMotionPrimitives::deltaHeadingNegRad);
}

  void SE2HAStarState::clear() {
    setXY(0,0); getMutableYaw() = 0; g_ = 0; h_ = 0;
    pred_.reset(); prim_ = 0; o_ = false; c_ = false;
    idx_ = -1;
  }
} // namespace Common
} // namespace HybridAStar