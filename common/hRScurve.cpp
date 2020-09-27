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
    double step_size = 0.2;
    Geometry::RSPath4Fork reedSheppPath(1/carPlant_->rad_, 0.1);

    double xy_resolution = 0.5, phi_resolution = 2*M_PI/72;
    std::vector<double> XYbound = {0,60,0,30};
    auto start = std::make_shared<Geometry::Node3d>(start_.getX(),start_.getY(),start_.getT(),xy_resolution,phi_resolution,XYbound);
    auto end = std::make_shared<Geometry::Node3d>(goal_.getX(),goal_.getY(),goal_.getT(),xy_resolution,phi_resolution,XYbound);

    return reedSheppPath.ShortestRSPlength(start,end);
  }
} // namespace Common
} // namespace HybridAStar
