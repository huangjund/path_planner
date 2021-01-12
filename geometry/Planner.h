#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <common/map/map.h>
#include <multibody/SingleForkLiftPlant.h>
#include <nav_msgs/OccupancyGrid.h>
#include "common/statespace/SE2State.h"

namespace HybridAStar {
namespace Geometry {
  class Planner {
  protected:
    std::shared_ptr<Common::SE2State> start_;
    Common::SE2State goal_;
    nav_msgs::OccupancyGrid::Ptr grid_;
  
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
  public:
    Planner() = default;
    explicit Planner(std::shared_ptr<Common::SE2State>,Common::SE2State&);
    virtual ~Planner() = default;
  
    Planner(const Planner &) = delete;
    Planner &operator=(const Planner &) = delete;
  };
} // namespace Geometry
} // namespace HybridAStar

#endif // PLANNER_H