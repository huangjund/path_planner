#include "SE2State.h"


namespace HybridAStar {
namespace Common {
  const int SE2State::dir = 3;
  const float SE2State::dy[] = { 0,        -0.2394141,  0.2394141};
  const float SE2State::dx[] = { 0.7,   0.6577848,   0.6577848};
  const float SE2State::dt[] = { 0,         0.34906585,   -0.34906585};

  const float SE2State::collisionMapCellSize = 1;

  SE2State::SE2State():SE2State(0,0,0,0,0,nullptr,0.5,0.08726646,0){}

  SE2State::SE2State(float cellsize, float anglesize):SE2State(0,0,0,0,0,nullptr,cellsize,anglesize,0){}

  SE2State::SE2State(float x, float y, float t, float g, float h,
                     std::shared_ptr<SE2State> pred, float cellsize, float anglesize, int prim = 0):
                     x_(x),y_(y),t_(t),g_(g),h_(h),prim_(prim),o_(false),c_(false),idx_(-1),
                    pred_(pred), cellSize_(cellsize), angleSize_(anglesize){
    setRelative(x,y,t); // set the relative position
  }

  SE2State::SE2State(const SE2State &cp):
                    x_(cp.x_),y_(cp.y_),t_(cp.t_),g_(cp.g_),h_(cp.h_),
                    prim_(cp.prim_),o_(cp.o_),c_(cp.c_),idx_(cp.idx_),pred_(cp.pred_),
                    cellSize_(cp.cellSize_),angleSize_(cp.angleSize_) {
    setRelative(x_,y_,t_);
  }

  SE2State &SE2State::operator=(const SE2State &rhs) {
    this->x_ = rhs.x_;this->y_ = rhs.y_; 
    this->t_ = rhs.t_; this->g_ = rhs.g_; 
    this->h_ = rhs.h_; this->prim_ = rhs.prim_; 
    this->o_ = rhs.o_; this->c_ = rhs.c_; 
    this->idx_ = rhs.idx_; this->pred_ = rhs.pred_;
    this->cellSize_ = rhs.cellSize_; this->angleSize_ = rhs.angleSize_;
    setRelative(x_,y_,t_);
    return *this;
  }

  void SE2State::freeState(State *state) const {
    auto *rstate = static_cast<SE2State *>(state);
    delete rstate;
  }

  bool SE2State::isOnGrid(const int width, const int height) const {
    return rx_ >= 0 && ry_ >= 0 && rx_ < width && ry_ < height;
  }

  bool SE2State::isInRange(const SE2State& goal) const {
    int random = rand() % 10 + 1;
    float dx = std::abs(x_ - goal.getX()) / random;
    float dy = std::abs(y_ - goal.getY()) / random;
    return (dx * dx) + (dy * dy) < dubinsShotDistance;
  }

  SE2State *SE2State::createSuccessor(const int i, std::shared_ptr<SE2State>& self) {
    float xSucc;
    float ySucc;
    float tSucc;

    // calculate successor positions forward
    if (i < 3) {
      xSucc = x_ + dx[i] * cos(t_) - dy[i] * sin(t_);
      ySucc = y_ + dx[i] * sin(t_) + dy[i] * cos(t_);
      tSucc = HybridAStar::Utils::normalizeHeadingRad(t_ + dt[i]);
    }
    // backwards
    else {
      xSucc = x_ - dx[i - 3] * cos(t_) - dy[i - 3] * sin(t_);
      ySucc = y_ - dx[i - 3] * sin(t_) + dy[i - 3] * cos(t_);
      tSucc = HybridAStar::Utils::normalizeHeadingRad(t_ - dt[i - 3]);
    }
    return new SE2State(xSucc,ySucc,tSucc,g_,0,self,cellSize_,angleSize_,i);
  }

  void SE2State::updateG() {
  // forward driving
  if (prim_ < 3) {
    // penalize turning
    if (pred_->prim_ != prim_) {
      // penalize change of direction
      if (pred_->prim_ > 2) {
        g_ += dx[0] * penaltyTurning * penaltyCOD;
      } else {
        g_ += dx[0] * penaltyTurning;
      }
    } else {
      g_ += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred_->prim_ != prim_) {
      // penalize change of direction
      if (pred_->prim_ < 3) {
        g_ += dx[0] * penaltyTurning * penaltyReversing * penaltyCOD;
      } else {
        g_ += dx[0] * penaltyTurning * penaltyReversing;
      }
    } else {
      g_ += dx[0] * penaltyReversing;
    }
  }
}

  bool SE2State::operator==(const SE2State& rhs) const {
    return (int)rx_ == (int)rhs.rx_ &&
          (int)ry_ == (int)rhs.ry_ &&
          (std::abs(t_ - rhs.t_) <= deltaHeadingRad ||
            std::abs(t_ - rhs.t_) >= deltaHeadingNegRad);
}

  void SE2State::clear() {
    x_ = 0; y_ = 0; t_ = 0; g_ = 0; h_ = 0;
    pred_.reset(); prim_ = 0; o_ = false; c_ = false;
    idx_ = -1;
  }
}  // namespace Common
}  // namespace HybridAStar