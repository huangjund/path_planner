#include <memory>

#include <gflags/gflags.h>
#include <ros/ros.h>

#include "../common/map/map.h"
#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"
#include "../common/collision/clsDetection.h"
#include "../multibody/SingleForkLiftPlant.h"

namespace HybridAStar{
  using HybridAStar::Common::CollisionDetection;
  using HybridAStar::Common::GridState;
  using HybridAStar::Common::SE2State;
  using HybridAStar::Common::Map;
  using HybridAStar::Multibody::SingleForkLiftPlant;

  int do_main(){
    ros::NodeHandle rosHandler;
    ros::Subscriber subMap;
    std::unique_ptr<Map<GridState>> collisionMap = std::make_unique<Map<GridState>>();
    std::unique_ptr<Map<SE2State>> planningMap = std::make_unique<Map<SE2State>>();
    std::unique_ptr<CollisionDetection> configSpace = std::make_unique<CollisionDetection>(); // the grid is a nullptr
    std::unique_ptr<SingleForkLiftPlant> plant = std::make_unique<SingleForkLiftPlant>();
    
    
    return 0;
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  return HybridAStar::do_main();
}
