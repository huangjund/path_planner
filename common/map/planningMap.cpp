#include "planningMap.h"

namespace HybridAStar {
namespace Common {
  template <class T>
  PlanningMap<T>::PlanningMap(const nav_msgs::OccupancyGrid::Ptr map) :
      CollisionMap(map) {}
  
  template <class T>
  void PlanningMap<T>::setSS(unsigned int count) {
    statespace = std::vector<T>(count, T());
    //   statespace = std::vector<T>(pwidth*pheight*72, T());
    hasStateSpace_ = true;
  }

  template <class T>
  void PlanningMap<T>::resetSS() {
    assert(statespace.size());
    for (auto i = statespace.begin(); i < statespace.end(); ++i)
      i->clear();
    hasStateSpace_ = true;
  }

  // template class Map<GridState>;
  // template class Map<SE2State>;
} // namespace Common
} // namespace HybridAStar