#include <memory>

#include <gflags/gflags.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "common/map/map.h"
#include "common/statespace/GridState.h"
#include "common/statespace/SE2State.h"

namespace HybridAStar{
  using HybridAStar::Common::GridState;
  using HybridAStar::Common::SE2State;
  using HybridAStar::Common::Map;

  int do_main(){
    std::unique_ptr<ros::NodeHandle> rosHandler;
    std::unique_ptr<Map<GridState>> collisionMap;
    std::unique_ptr<Map<SE2State>> planningMap;
    ros::Subscriber subMap;
    
    // the map is subscribed by object, collision map 
    subMap = rosHandler->subscribe<const nav_msgs::OccupancyGrid::Ptr, Map<GridState>>(
                                      "/map", 1, &Map<GridState>::setMap, collisionMap.get());
    return 0;
  }
}
int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return HybridAStar::do_main();
}
