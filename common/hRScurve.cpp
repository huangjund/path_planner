#include "hRScurve.h"

namespace HybridAStar {
namespace Common {
  hRScurve::hRScurve(SE2State &start, SE2State &goal): start_(start), goal_(goal){}

  void hRScurve::setStart(SE2State &start) {
    start_ = start;
  }

  void hRScurve::setGoal(SE2State &goal) {
    goal_ = goal;
  }

  void hRScurve::setStartGoal(SE2State &start, SE2State &goal) {
    start_ = start;
    goal_ = goal;
  }

  double hRScurve::getDistance() {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(carPlant_->rad_);
    std::unique_ptr<ompl::base::SE2StateSpace::StateType> rsStart(reedsSheppPath.allocState()->as<ompl::base::SE2StateSpace::StateType>());
    std::unique_ptr<ompl::base::SE2StateSpace::StateType> rsEnd(reedsSheppPath.allocState()->as<ompl::base::SE2StateSpace::StateType>());
    rsStart->setXY(start_.getX(), start_.getY());
    rsStart->setYaw(start_.getT());
    rsEnd->setXY(goal_.getX(), goal_.getY());
    rsEnd->setYaw(goal_.getT());

    return reedsSheppPath.distance(rsStart.get(), rsEnd.get());
  }
} // namespace Common
} // namespace HybridAStar
