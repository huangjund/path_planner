#include "visualize.h"

#include <limits>
using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {
  poses3D.poses.clear();
  poses3Dreverse.poses.clear();

  // 3D COSTS
  visualization_msgs::MarkerArray costCubes3D;
  visualization_msgs::Marker costCube3D;
  // CLEAR THE COST HEATMAP
  costCube3D.header.frame_id = "path";
  costCube3D.header.stamp = ros::Time::now();
  costCube3D.id = 0;
  costCube3D.action = 3;
  costCubes3D.markers.push_back(costCube3D);
  pubNodes3DCosts.publish(costCubes3D);
}

//###################################################
//                                    CURRENT 3D NODE
//###################################################
void Visualize::publishNode3DPose(SE2State& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX();
  pose.pose.position.y = node.getY();

  //FORWARD
  if (node.getPrim() < 3) {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }
  //REVERSE
  else {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
  }

  // PUBLISH THE POSE
  pubNode3D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 3D NODES
//###################################################
void Visualize::publishNode3DPoses(SE2State& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX();
  pose.position.y = node.getY();

  //FORWARD
  if (node.getPrim() < 3) {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    poses3D.poses.push_back(pose);
    poses3D.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes3D.publish(poses3D);
  }
  //REVERSE
  else {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    poses3Dreverse.poses.push_back(pose);
    poses3Dreverse.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes3Dreverse.publish(poses3Dreverse);
  }

}

//###################################################
//                                    COST HEATMAP 3D
//###################################################
void Visualize::publishNode3DCosts(std::shared_ptr<Common::Map<SE2State>> pmap,int depth) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::min();
  int idx;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int width = pmap->info_.width*pmap->info_.resolution/pmap->info_.planResolution;
  int height = pmap->info_.height*pmap->info_.resolution/pmap->info_.planResolution;
  

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // ________________________________
  // DETERMINE THE MAX AND MIN VALUES
  for (int i = 0; i < width * height; ++i) {
    values[i] = 0;

    // iterate over all headings
    for (int k = 0; k < depth; ++k) {
      idx = k * width * height + i;

      // set the minimum for the cell
      if (pmap->statespace[idx].isClosed() || pmap->statespace[idx].isOpen()) {
        values[i] = pmap->statespace[idx].getC();
      }
    }

    // set a new minimum
    if (values[i] > 0 && values[i] < min) {
      min = values[i];
    }

    // set a new maximum
    if (values[i] > 0 && values[i] > max) {
      max = values[i];
    }
  }

  // _______________
  // PAINT THE CUBES
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (values[i]) {

      // delete all previous markers
      if (once) {
        costCube.action = visualization_msgs::Marker::DELETEALL;
        once = false;
      } else {
        costCube.action = visualization_msgs::Marker::ADD;
      }


      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      // TODO: as here needs cell size, while cell size should be directly related with 
      // map, so this class needs to be differentiated
      float planMapSize = 0.5;
      costCube.scale.x = planMapSize;
      costCube.scale.y = planMapSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      // TODO: not sure about this scale
      costCube.pose.position.x = (i % width + 0.5) * planMapSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * planMapSize;
      costCubes.markers.push_back(costCube);
    }
  }

  // PUBLISH THE COSTCUBES
  pubNodes3DCosts.publish(costCubes);
}

