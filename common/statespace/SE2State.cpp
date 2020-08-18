#include "SE2State.h"


namespace HybridAStar {
namespace Common {
  const int SE2State::dir = 3;
  const float SE2State::dy[] = { 0,        -0.0415893,  0.0415893};
  const float SE2State::dx[] = { 0.7068582,   0.705224,   0.705224};
  const float SE2State::dt[] = { 0,         0.1178097,   -0.1178097};

  const float SE2State::collisionMapCellSize = 1;

  void SE2State::freeState(State *state) const {
    auto *rstate = static_cast<SE2State *>(state);
    delete rstate;
  }

  inline bool SE2State::isOnGrid(const int width, const int height) const {
    return rx_ >= 0 && ry_ >= 0 && rx_ < width && ry_ < height;
  }

  SE2State *SE2State::createSuccessor(const int i) {
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
    return new SE2State(xSucc,ySucc,tSucc,g_,0,this,cellSize_,angleSize_,i);
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
}  // namespace Common
}  // namespace HybridAStar