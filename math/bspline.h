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

/**
 * @brief get a piece of smooth b spline
 * 
 */
class BSpline : public Smoother {
  public:
    /**
     * @brief control point structure
     * 
     */
    struct ctrlPoint {
      double value;
      int multiplicity = 1;
      ctrlPoint() = default;
      ctrlPoint(const ctrlPoint& c):value(c.value),multiplicity(c.multiplicity){}
      ctrlPoint& operator=(const ctrlPoint& c) {value = c.value;  multiplicity = c.multiplicity; return *this;}
    };

  private:
    std::vector<std::vector<ctrlPoint>> ctrlPointSet; ///< control point set. the size should = trajPointSet size = splineOrder size
    std::vector<std::vector<Vector2D>> trajPointSet;  ///< trajectory point set
    std::vector<std::vector<Vector2D>> outputTrajPointSet;  ///< this vector is for agv, not for inner planning
    std::vector<bool> trajDirctionSet; ///< if is forward, true, if is backward, false;
    std::vector<unsigned int> splineOrder;  ///< order of every piece of spline
    std::vector<std::shared_ptr<Common::SE2State>> bpath_;

    void addPointBetweenLine();

  public:
    BSpline();
    virtual ~BSpline();
    
    /**
     * @brief trace the path from the end point of the path
     * 
     * @param node end point
     */
    virtual void tracePath(const std::shared_ptr<Common::SE2State> node);

    /**
     * @brief smooth the traced path
     * 
     * @param width 
     * @param height 
     */
    virtual void smoothPath(float width, float height);

    /**
     * @brief whether it is a cusp point
     * 
     * @param i 
     * @return true 
     * @return false 
     */
    virtual bool isCusp(int i);
    virtual void clearPath();

    /**
     * @brief initialize orders of the traced path
     * 
     * @param autogen auto generate
     */
    void initializeSplineOrders(bool autogen = true);

    /**
     * @brief Set the control Points of every traced path
     * 
     * @param autogen 
     */
    void setCtrlPoints(bool autogen = true);
    std::vector<std::vector<Vector2D>>& returnTrajSet() {return outputTrajPointSet;}
    std::vector<bool>& returnDirectionSet() {return trajDirctionSet;}
    
  //visualization
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  nav_msgs::Path path;

  public:
  /**
   * @brief for ros visualization
   * 
   */
  void interpolate();

  /**
   * @brief for ros visualization
   * 
   */
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