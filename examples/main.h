#pragma once 

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "../common/map/map.h"
#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"
#include "../common/collision/clsDetection.h"
#include "../multibody/SingleForkLiftPlant.h"
#include "geometry/hybridastar/HAstar.h"

namespace HybridAStar{
  class Interface
  {
  private:
    ros::NodeHandle n_;
    ros::Subscriber subStart_;
    ros::Subscriber subGoal_;
    ros::Subscriber subMap_;
    bool isStartvalid_;
    bool isGoalvalid_;
    // raw datas from outside
    geometry_msgs::PoseWithCovarianceStamped start_;
    geometry_msgs::PoseStamped goal_;
    nav_msgs::OccupancyGrid::Ptr grid_;
  public:
    explicit Interface();
    ~Interface();

    void makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );
    void makeGoal(const geometry_msgs::PoseStamped::ConstPtr& );
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);
  };
}