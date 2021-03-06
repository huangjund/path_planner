#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "math/gradient.h"

#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"
#include "../common/map/map.h"

namespace HybridAStar {
using Common::SE2State;
using namespace visualization_msgs;
//class Common::GridState;
/*!
   \brief A class for visualizing the hybrid A* search.

  Depending on the settings in constants.h the visualization will send different amounts of detail.
  It can show the 3D search as well as the underlying 2D search used for the holonomic with obstacles heuristic.
*/
class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  /// The default constructor initializing the visualization object and setting publishers for the same.
  Visualize() {
    // _________________
    // TOPICS TO PUBLISH
    pubNode3D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);
    pubNodes3Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPosesReverse", 100);
    pubNodes3DCosts = n.advertise<MarkerArray>("/visualizeNodes3DCosts", 1000);

    // CONFIGURE THE CONTAINER
    poses3D.header.frame_id = "path";
    poses3Dreverse.header.frame_id = "path";
    poses2D.header.frame_id = "path";
  }

  // CLEAR VISUALIZATION
  /// Clears the entire visualization
  void clear();

  // PUBLISH A SINGLE/ARRAY 3D NODE TO RViz
  /// Publishes a single node to RViz, usually the one currently being expanded
  void publishNode3DPose(SE2State& node);
  /// Publishes all expanded nodes to RViz
  void publishNode3DPoses(SE2State& node);
  // PUBLISH THE COST FOR A 3D NODE TO RViz
  /// Publishes the minimum of the cost of all nodes in a 2D grid cell
  void publishNode3DCosts(std::shared_ptr<Common::Map<SE2State>> pmap, int depth = 72);

 private:
  /// A handle to the ROS node
  ros::NodeHandle n;
  /// Publisher for a single 3D node
  ros::Publisher pubNode3D;
  /// Publisher for an array of 3D forward nodes
  ros::Publisher pubNodes3D;
  /// Publisher for an array of 3D reaward nodes
  ros::Publisher pubNodes3Dreverse;
  /// Publisher for an array of 3D cost with color gradient
  ros::Publisher pubNodes3DCosts;
  /// Array of poses describing forward nodes
  geometry_msgs::PoseArray poses3D;
  /// Array of poses describing reaward nodes
  geometry_msgs::PoseArray poses3Dreverse;
  /// Array of poses describing 2D heuristic nodes
  geometry_msgs::PoseArray poses2D;
};
}
#endif // VISUALIZE_H
