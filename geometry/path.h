#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "../common/statespace/SE2State.h"
#include "../utils/Helper.h"
#include "../multibody/SingleForkLiftPlant.h"

namespace HybridAStar {
/*!
   \brief A class for tracing and visualizing the path generated by the Planner
*/
class Path {
 public:
  /// The default constructor initializing the path object and setting publishers for the same.
  Path(bool smoothed = false) {
    std::string pathTopic = "/path";
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";

    if (smoothed) {
      pathTopic = "/sPath";
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      this->smoothed = smoothed;
    }

    // _________________
    // TOPICS TO PUBLISH
    pubPath = n.advertise<nav_msgs::Path>(pathTopic, 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1);

    // CONFIGURE THE CONTAINER
    path.header.frame_id = "path";
  }

  //  // __________
  //  // TRACE PATH
  //  /*!
  //     \brief Given a node pointer the path to the root node will be traced recursively
  //     \param node a 3D node, usually the goal node
  //     \param i a parameter for counting the number of nodes
  //  */
  //  void tracePath(const Common::SE2State* node, int i = 0);
  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  void updatePath(std::vector<Common::SE2State> nodePath);
  /*!
     \brief Adds a segment to the path
     \param node a 3D node
  */
  void addSegment(const Common::SE2State& node);
  /*!
     \brief Adds a node to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
  */
  void addNode(const Common::SE2State& node, int i);
  /*!
     \brief Adds a vehicle shape to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
  */
  void addVehicle(const Common::SE2State& node, int i);

  // ______________
  // PUBLISH METHODS

  /// Clears the path
  void clear();
  /// Publishes the path
  void publishPath() { pubPath.publish(path); }
  /// Publishes the nodes of the path
  void publishPathNodes() { pubPathNodes.publish(pathNodes); }
  /// Publishes the vehicle along the path
  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }

 private:
  std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
  /// A handle to the ROS node
  ros::NodeHandle n;
  /// Publisher for the path as a spline
  ros::Publisher pubPath;
  /// Publisher for the nodes on the path
  ros::Publisher pubPathNodes;
  /// Publisher for the vehicle along the path
  ros::Publisher pubPathVehicles;
  /// Path data structure for visualization
  nav_msgs::Path path;
  /// Nodes data structure for visualization
  visualization_msgs::MarkerArray pathNodes;
  /// Vehicle data structure for visualization
  visualization_msgs::MarkerArray pathVehicles;
  /// Value that indicates that the path is smoothed/post processed
  bool smoothed = false;
};

namespace Constants {
   // ____________________________________________
   // COLOR DEFINITIONS FOR VISUALIZATION PURPOSES
   /// A structure to express colors in RGB values
   struct color {
   /// the red portion of the color
   float red;
   /// the green portion of the color
   float green;
   /// the blue portion of the color
   float blue;
   };
   /// A definition for a color used for visualization
   static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};
   /// A definition for a color used for visualization
   static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};
   /// A definition for a color used for visualization
   static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};
   /// A definition for a color used for visualization
   static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};
   /// A definition for a color used for visualization
   static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
}  // namespace Constants
}
#endif // PATH_H
