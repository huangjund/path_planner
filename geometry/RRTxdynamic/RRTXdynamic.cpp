#include "RRTXdynamic.h"

namespace HybridAStar
{
namespace Geometry
{
  RRTXdynamic::RRTXdynamic(const point_t& goal, const point_t& start):
                          v_goal_(goal),v_start_(start),v_bot_(start),
                          vertexSet_(pointVec{v_goal_}),
                          visibleTree_(kdtree(vertexSet_)) {

  }

  void RRTXdynamic::solve(const Common::PlannerTerminationCondition& ptc) {
    while(1) {
      r_ = shrinkingBallRadius(vertexSet_.size());
      updateObstacles();
      auto v = genRandom();
      auto v_nearest = nearest(v);

    }
  }

  void RRTXdynamic::solve(double solvetime) {
    if (solvetime < 1.0)
      return solve(Common::timedPlannerTerminationCondition(solvetime));
    return solve(Common::timedPlannerTerminationCondition(solvetime, std::min(solvetime / 100.0, 0.1)));
  }

  double RRTXdynamic::shrinkingBallRadius(const size_t& vsetSize) {
    return static_cast<double>(vsetSize);
  }


  void RRTXdynamic::updateObstacles() {
  }

  point_t RRTXdynamic::genRandom() {}

  point_t RRTXdynamic::nearest(const point_t& v) {}
} // namespace Geometry
} // namespace HybridAStar
