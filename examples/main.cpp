#include <memory>

#include <gflags/gflags.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "../common/map/map.h"
#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"

namespace HybridAStar{
  using HybridAStar::Common::GridState;
  using HybridAStar::Common::SE2State;
  using HybridAStar::Common::Map;

  int do_main(){
    ros::NodeHandle rosHandler;
    ros::Subscriber subMap;
    std::unique_ptr<Map<GridState>> collisionMap = std::make_unique<Map<GridState>>();
    // std::unique_ptr<Map<SE2State>> planningMap = std::make_unique<Map<SE2State>>();
    
    return 0;
  }
}
int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  return HybridAStar::do_main();
}
