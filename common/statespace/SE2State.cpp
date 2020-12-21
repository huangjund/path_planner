#include "SE2State.h"

namespace HybridAStar {
namespace Common {

SE2State::SE2State() {
  addState(std::make_shared<RealVectorState<double>>(2u));
  addState(std::make_shared<SO2State>());
}

SE2State::SE2State(const SE2State& se2){
  components[0] = se2.components[0];
  components[1] = se2.components[1];
}

SE2State& SE2State::operator=(const SE2State& rhs) {
  components[0] = rhs.components[0];
  components[1] = rhs.components[1];
  return *this;
}
}  // namespace Common
}  // namespace HybridAStar