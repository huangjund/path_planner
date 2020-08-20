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
    Common::SE2State start_;
    Common::SE2State goal_;
    // TODO: change this into map class
    nav_msgs::OccupancyGrid::Ptr grid_;
  
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
  public:
    explicit Planner(Common::SE2State&,Common::SE2State&);
    virtual ~Planner() = default;

    Planner(const Planner &) = delete;
    Planner &operator=(const Planner &) = delete;
  };
} // namespace Geometry
} // namespace HybridAStar

#endif // PLANNER_H