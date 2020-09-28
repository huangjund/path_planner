#include "RRTXdynamic.h"

namespace HybridAStar
{
namespace Geometry
{
  RRTXdynamic::RRTXdynamic(const Vertex& goal, const Vertex& start):
                          v_goal_(goal),v_start_(start),v_bot_(start) {}

  void RRTXdynamic::solve(const Common::PlannerTerminationCondition& ptc) {
    while(1) {
      r_ = shrinkingBallRadius(vertexSet_.size());
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

} // namespace Geometry
} // namespace HybridAStar



// int main() {
//     pointVec points;
//     point_t pt;

//     pt = {0.0, 0.0};
//     points.push_back(pt);
//     pt = {1.0, 0.0};
//     points.push_back(pt);
//     pt = {0.0, 1.0};
//     points.push_back(pt);
//     pt = {1.0, 1.0};
//     points.push_back(pt);
//     pt = {0.5, 0.5};
//     points.push_back(pt);

//     KDTree tree(points);

//     std::cout << "nearest test\n";
//     pt = {0.8, 0.2};
//     auto res = tree.nearest_point(pt);
//     for (double b : res) {
//         std::cout << b << " ";
//     }
//     std::cout << '\n';

//     auto res2 = tree.neighborhood_points(pt, .55);

//     for (point_t a : res2) {
//         for (double b : a) {
//             std::cout << b << " ";
//         }
//         std::cout << '\n';
//     }
//     return 0;
// }