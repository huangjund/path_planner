#ifndef _HYBRID_A_STAR_SO_2_STATE_H_
#define _HYBRID_A_STAR_SO_2_STATE_H_

#include "State.h"
#include "utils/Helper.h"

namespace HybridAStar {
namespace Common {
class SO2State : public State {
 public:
  SO2State() = default;
  virtual ~SO2State() override = default;

  SO2State(const SO2State& state):radian_(state.radian_) {}
  SO2State& operator=(const SO2State& state) { radian_ = state.radian_; return *this;}
  
  double getR() const {
    return Utils::normalizeHeadingRad(radian_);
  }

  double& getMutableR() {
    return radian_;
  }
 protected:
  // (0, 2pi]
  double radian_ = 0;
};
} // namespace Common
} // namespace HybridAStar
#endif