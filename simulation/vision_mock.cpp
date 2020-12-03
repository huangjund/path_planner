#include <memory>
#include <math.h>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PoseStamped.h>


namespace HybridAStar {
namespace examples {
using namespace visualization_msgs;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
ros::Publisher obstacle_Pos_Mock;

// create a box
Marker makeBox() {
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = 1.2;
  marker.scale.y = 0.9;
  marker.scale.z = 0.8;
  marker.color.a = 1;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;

  return marker;
}

void confirmPos(const InteractiveMarkerFeedbackConstPtr& feedback) {
  return;
}

void addBox(const InteractiveMarkerFeedbackConstPtr&) {
  void makeMutableObstacle(const tf::Vector3&);

  tf::Vector3 position(60, 5, 0.4);
  makeMutableObstacle(position);
  
  return;
}

void processFeedback(const InteractiveMarkerFeedbackConstPtr& feedback) {
  switch(feedback->event_type) {
    case InteractiveMarkerFeedback::MOUSE_UP:
      geometry_msgs::PoseStamped pose;
      pose.pose.position = feedback->pose.position;
      pose.pose.orientation = feedback->pose.orientation;
      pose.header = feedback->header;
      pose.header.frame_id = feedback->marker_name;
      obstacle_Pos_Mock.publish(pose);
      break;
  }
  return;
}

void makeControlVis(InteractiveMarker& marker, Marker& entity) {
  InteractiveMarkerControl control;
  control.name = "control_visualization";
  control.always_visible = true;
  control.markers.push_back( entity );
  marker.controls.push_back( control );
}

void makeMenu(const tf::Vector3& position) {
  menu_handler.insert( "add box", &addBox );
  menu_handler.insert( "confirm position", &confirmPos);

  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;
  int_marker.name = "menu";
  int_marker.description = "MENU";

  InteractiveMarkerControl control;
  control.name = "menu_choose";
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;

  auto mark = makeBox();
  control.always_visible = true;
  control.markers.push_back(mark);
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  menu_handler.apply(*server, int_marker.name);
}

void makeMutableObstacle(const tf::Vector3& position) {
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  static int count = 0;
  std::ostringstream oss;
  oss << "obstacle" << count++;
  int_marker.name = oss.str();

  std::cout << int_marker.name << std::endl;

  auto mark = makeBox();
  makeControlVis(int_marker, mark);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  server->applyChanges();
}

} // namespace examples
} // namespace HybridAStar


int main (int argc, char** argv) {
  using namespace HybridAStar::examples;

  ros::init(argc, argv, "Sensor_Mock");
  ros::NodeHandle n;

  server.reset( new interactive_markers::InteractiveMarkerServer("Sensor_Mock","",false) );
  obstacle_Pos_Mock = n.advertise<geometry_msgs::PoseStamped>("/sensor/obstaclePos", 1);

  ros::Duration(0.1).sleep();

  tf::Vector3 position(60,0,0.4);
  makeMenu(position);

  server->applyChanges();

  ros::spin();

  server.reset();
  return 0;
}