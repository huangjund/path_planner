#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <common/map/map.h>
#include <multibody/SingleForkLiftPlant.h>

namespace HybridAStar {
namespace Geometry {
  class Planner {
  private:
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
    ros::NodeHandle n_;
    ros::Subscriber subStart_;
    ros::Subscriber subGoal_;
    bool isStartvalid = false;
    bool isGoalvalid = false;
    geometry_msgs::PoseWithCovarianceStamped start_;
    geometry_msgs::PoseStamped goal_;


    void makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& );
    void makeGoal(const geometry_msgs::PoseStamped::ConstPtr& );
  public:
    explicit Planner();
    virtual ~Planner() = default;

    Planner(const Planner &) = delete;
    Planner &operator=(const Planner &) = delete;


  };
} // namespace Geometry
} // namespace HybridAStar

#endif // PLANNER_H