#include <memory>

#include "common/hRScurve.h"
#include "utils/Helper.h"
#include "common/statespace/GridState.h"

using namespace HybridAStar;

int main() {
  auto start = std::make_shared<SE2State>(0.5,0.08726646);
  Common::SE2State goal(0.5,0.08726646);
  start->setX(0);
  start->setY(12.1);
  goal.setX(10.1);
  goal.setY(123);
  start->setT(Utils::normalizeHeadingRad(2);
  goal.setT(Utils::normalizeHeadingRad(3);
  auto f = new Common::hRScurve(*start, goal);
  return 0;
}