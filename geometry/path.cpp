#include "path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Common::SE2StatePtr node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  // addNode(node, 0);
  // addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Common::SE2State* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Common::SE2StatePtr> nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (auto i = nodePath.cbegin(); i != nodePath.cend(); ++i) {
    addSegment(*i);
    addNode(*i, k);
    k++;
    addVehicle(*i, k);
    k++;
  }
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Common::SE2StatePtr& node) {
  geometry_msgs::PoseStamped vertex;
  // TODO: should this x multiply a collision cell size
  vertex.pose.position.x = node->getX();
  vertex.pose.position.y = node->getY();
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Common::SE2StatePtr& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node->getX();
  pathNode.pose.position.y = node->getY();
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Common::SE2StatePtr& node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Common::ForkProperty::length_ - Common::ForkProperty::bloating_ * 2;
  pathVehicle.scale.y = Common::ForkProperty::width_ - Common::ForkProperty::bloating_ * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node->getX();
  pathVehicle.pose.position.y = node->getY();
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node->getYaw());
  pathVehicles.markers.push_back(pathVehicle);
}
