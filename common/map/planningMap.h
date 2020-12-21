#ifndef _HYBRID_A_STAR_PLANNING_MAP_H_
#define _HYBRID_A_STAR_PLANNING_MAP_H_

#include "collisionMap.h"

#include "common/parameters/parameters.h"

namespace HybridAStar {
namespace Common {

template <class T>
class PlanningMap : public CollisionMap {
 private:
  bool hasStateSpace_ = false;
  float planResolution; // [unit: meters/cell]
  std::vector<T> statespace;
 public:
  PlanningMap() = default;
  explicit PlanningMap(const nav_msgs::OccupancyGrid::Ptr& map);
  virtual ~PlanningMap() override = default;

  // non-copyable
  PlanningMap(const PlanningMap<T>& map) = delete;
  PlanningMap<T>& operator=(const PlanningMap<T>& map) = delete;

  void setSS(unsigned int count);
  void resetSS();
  // const std::vector<T>& getStateSpace() const;
  const bool hasStateSpace() { return hasStateSpace_;}

};
} // namespace Common
} // namespace HybridAStar
#endif