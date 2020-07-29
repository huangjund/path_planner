#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." <<Constants::coutDEBUG<< std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  // for collision detection preparation
  configurationSpace.updateGrid(map);

  //create array for Voronoi diagram
  int collisionMapHeight = map->info.height;
  int collisionMapWidth = map->info.width;
  bool** binMap;
  binMap = new bool*[collisionMapWidth];

  for (int x = 0; x < collisionMapWidth; x++) { binMap[x] = new bool[collisionMapHeight]; }

  for (int x = 0; x < collisionMapWidth; ++x) {
    for (int y = 0; y < collisionMapHeight; ++y) {
      binMap[x][y] = map->data[y * collisionMapWidth + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(collisionMapWidth, collisionMapHeight, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::Rad2Deg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::Rad2Deg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::Rad2Deg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::Rad2Deg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // get two maps' property
    constexpr int collisionMapWidth = grid->info.width;
    constexpr int collisionMapHeight = grid->info.height;
    constexpr int depth = Constants::vehicleHeadings;
    constexpr int clisLength = collisionMapWidth * collisionMapHeight * depth; // the collision map length
    constexpr int planMapWidth = static_cast<int>(
      collisionMapWidth*Constants::collisionMapCellSize/PlanMapNode::planmapCellSize);
    constexpr int planMapHeight = static_cast<int>(
      collisionMapHeight*Constants::collisionMapCellSize/PlanMapNode::planmapCellSize);
    constexpr int planLength = planMapWidth*planMapHeight*depth; // the planning map length
    // create two planning maps
    PlanMapNode* nodes3D = new PlanMapNode[planLength]();
    Node2D* nodes2D = new Node2D[planMapWidth*planMapHeight]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x/PlanMapNode::planmapCellSize;
    float y = goal.pose.position.y/PlanMapNode::planmapCellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,360]
    t = Helper::Rad2Deg(t);
    const PlanMapNode nGoal(x, y, t, 0, 0, nullptr);

    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x/PlanMapNode::planmapCellSize;
    y = start.pose.pose.position.y/PlanMapNode::planmapCellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,360]
    t = Helper::Rad2Deg(t);
    PlanMapNode nStart(x, y, t, 0, 0, nullptr);

    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    ros::Time t1 = ros::Time::now();
    // FIND THE PATH
    PlanMapNode* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, planMapWidth, planMapHeight, configurationSpace, dubinsLookup, visualization);
    ros::Time t2 = ros::Time::now();
    // TRACE THE PATH
    smoother.tracePath(nSolution);
    ros::Time t3 = ros::Time::now();
    // CREATE THE UPDATED PATH
    path.updatePath(smoother.getPath());
    ros::Time t4 = ros::Time::now();
    // SMOOTH THE PATH
    smoother.smoothPath(voronoiDiagram); // TODO: need to know
    ros::Time t5 = ros::Time::now();
    // CREATE THE UPDATED PATH
    smoothedPath.updatePath(smoother.getPath());
    ros::Time t6 = ros::Time::now();
    ros::Duration d1(t1 - t0);ros::Duration d2(t2 - t1);ros::Duration d3(t3 - t2);ros::Duration d4(t4 - t3);ros::Duration d5(t5 - t4);
    ros::Duration d(t6 - t0);
    std::cout << "TIME in ms: \n" << d1 * 1000 << "\t"
                                  << d2 * 1000 << "\t"
                                  << d3 * 1000 << "\t"
                                  << d4 * 1000 << "\t"
                                  << d5 * 1000 << "\t"
                                  << d * 1000 << "\t" << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    visualization.publishNode3DCosts(nodes3D, collisionMapWidth, collisionMapHeight, depth);
    visualization.publishNode2DCosts(nodes2D, collisionMapWidth, collisionMapHeight);



    delete [] nodes3D;
    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
