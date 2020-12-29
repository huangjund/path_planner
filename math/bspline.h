#ifndef _HYBRID_A_STAR_B_SPLINE_H
#define _HYBRID_A_STAR_B_SPLINE_H

#include "smoother.h"
#include "vector2d.h"
#include "../common/statespace/SE2State.h"
#include "bsplinebasis.h"

#include <vector>
#include <cassert>
#include <memory>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

namespace HybridAStar
{
class BSpline : public Smoother {
  public:
    struct ctrlPoint {
      double value;
      int multiplicity = 1;
      ctrlPoint() = default;
      ctrlPoint(const ctrlPoint& c):value(c.value),multiplicity(c.multiplicity){}
      ctrlPoint& operator=(const ctrlPoint& c) {value = c.value;  multiplicity = c.multiplicity; return *this;}
    };

  private:
    std::vector<std::vector<ctrlPoint>> ctrlPointSet; // the size should = trajPointSet size = splineOrder size
    std::vector<std::vector<Vector2D>> trajPointSet;
    std::vector<std::vector<Vector2D>> outputTrajPointSet;  // this vector is for agv, not for inner planning
    std::vector<bool> trajDirctionSet; // if is forward, true, if is backward, false;
    std::vector<unsigned int> splineOrder;
    std::vector<std::shared_ptr<Common::SE2State>> bpath_;

    void addPointBetweenLine();

  public:
    BSpline();
    virtual ~BSpline();
    
    virtual void tracePath(const std::shared_ptr<Common::SE2State> node);
    virtual void smoothPath(float width, float height);
    virtual bool isCusp(int i);
    virtual void clearPath();
    void initializeSplineOrders(bool autogen = true);
    void setCtrlPoints(bool autogen = true);
    std::vector<std::vector<Vector2D>>& returnTrajSet() {return outputTrajPointSet;}
    std::vector<bool>& returnDirectionSet() {return trajDirctionSet;}
    
    //visualization
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  nav_msgs::Path path;

  public:
  void interpolate();
  void splinePub() {
    path.poses.clear();
    bpath_.clear();
    interpolate();

    path.header.stamp = ros::Time::now();
    for (auto i = bpath_.cbegin(); i != bpath_.end(); i++)
    {
      geometry_msgs::PoseStamped vertex;
      vertex.pose.position.x = (*i)->getX();
      vertex.pose.position.y = (*i)->getY();
      vertex.pose.position.z = 0;
      vertex.pose.orientation.w = 1;
      vertex.pose.orientation.x = 0;
      vertex.pose.orientation.y = 0;
      vertex.pose.orientation.z = 0;
      path.poses.push_back(vertex);
    }

    pub.publish(path);
  }
}; // class bspline
} // namespace HybridAStar

#endif