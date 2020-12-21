#ifndef _HYBRID_A_STAR_SE2_STATE_H_
#define _HYBRID_A_STAR_SE2_STATE_H_

#include "State.h"
#include "RealVectorState.h"
#include "SO2State.h"

#include "utils/PtrWrapper.h"

namespace HybridAStar {
namespace Common {
  CLASS_SHARED(SE2State);
  // the SE2State class contains discretization information
  class SE2State : public CompoundState {
   public:
    SE2State();
    virtual ~SE2State() override = default;

    SE2State(const SE2State& se2);
    SE2State& operator=(const SE2State& se2);

    double getX() const {
      return as<RealVectorState<double>>(0)->values_[0];
    }
    double& getMutableX() {
      return as<RealVectorState<double>>(0)->values_[0];
    }
    double getY() const {
      return as<RealVectorState<double>>(0)->values_[1];
    }
    double& getMutableY() {
      return as<RealVectorState<double>>(0)->values_[1];
    }
    double getYaw() const {
      return as<SO2State>(1)->getR();
    }
    double& getMutableYaw() {
      return as<SO2State>(1)->getMutableR();
    }
    void setXY(double x, double y) {
      auto xy = as<RealVectorState<double>>(0)->values_;
      xy[0] = x; xy[1] = y;
    }
    void setXY(double* v) {
      auto xy = as<RealVectorState<double>>(0)->values_;
      xy[0] = v[0]; xy[1] = v[1];
    }
  };
} // namespace Common
} // namespace HybridAStar

#endif