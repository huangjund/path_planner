#include <memory>

#include <gflags/gflags.h>

#include "main.h"

namespace HybridAStar{

  Interface::Interface():automata_(initial),
    planningMap(std::make_shared<Map<SE2State>>()),
    configSpace(std::make_shared<CollisionDetection>()),
    planner_(std::make_unique<HAstar>())
  {
    isStartvalid_ = false;
    isGoalvalid_ = false;
    hasMap_ = false;
    pubPath_ = n_.advertise<std_msgs::String>("/PlannerToinServer/Path", 1);
    posClient_ = n_.serviceClient<gazebo_msgs::GetModelState>("/inServerToPlanner/Pos");
    subGoal_ = n_.subscribe("/move_base_simple/goal", 1, &Interface::makeGoal, this);
    subStart_ = n_.subscribe("/initialpose", 1, &Interface::makeStart, this);
    subMap_ = n_.subscribe("/map", 1, &Interface::setMap, this);
    subObs_ = n_.subscribe("/sensor/obstaclePos", 1, &Interface::updateObs, this);
    tempSubscriber_ = n_.advertise<geometry_msgs::PoseStamped>("/PlannerToRVIZ/Target",1);
  }

  void Interface::makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
    // TODO: lack of a solution to no map loaded
    float x = start->pose.pose.position.x / grid_->info.resolution;   //[unit: collision cells]
    float y = start->pose.pose.position.y / grid_->info.resolution;
    float t = tf::getYaw(start->pose.pose.orientation);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
      isStartvalid_ = true;
      Action action = startSetting;
      start_ = *start;
      
      // automata
      outputAutomata(action);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }

    
  }

  void Interface::makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    // retrieving goal position
    float x = goal->pose.position.x / grid_->info.resolution; // [unit:collision cells]
    float y = goal->pose.position.y / grid_->info.resolution;
    float t = tf::getYaw(goal->pose.orientation);

    // {
    //   gazebo_msgs::GetModelState srv;
    //   srv.request.model_name = "goalpose";
    //   if (posClient_.call(srv)){
    //     geometry_msgs::PoseStamped goalOutput;
    //     goalOutput.header.frame_id = "map";
    //     goalOutput.header.stamp = ros::Time::now();
    //     goalOutput.pose.position.x = srv.response.pose.position.x;
    //     goalOutput.pose.position.y = srv.response.pose.position.y;
    //     goalOutput.pose.position.z = 0;
    //     goalOutput.pose.orientation = tf::createQuaternionMsgFromYaw(srv.response.pose.position.z);
    //     tempSubscriber_.publish(goalOutput);
    //   }
    //   else {
    //     std::cout << "failed to get response" << std::endl;
    //   }
    // }

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0) {
      isGoalvalid_ = true;
      Action action = goalSetting;
      goal_ = *goal;

      // automata
      outputAutomata(action);
    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }
  
  void Interface::setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    grid_ = map;
    hasMap_ = true;
    Action action = Action::mapSetting;
    clearStartandGoal();
    
    // automata
    outputAutomata(action);
  }

  void Interface::updateObs(const geometry_msgs::PoseStampedPtr& pose) {
    // get basic pose information
    std::array<double,4> info;
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(pose->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

    int id = pose->header.frame_id.back() - '0';
    info[0] = pose->pose.position.x;
    info[1] = pose->pose.position.y;
    info[2] = pose->pose.position.z;
    info[3] = yaw;

    // project obstacle from cooridnate to blocks
    std::unordered_map<int, bool> obstacle;
    obsProjection(info, obstacle);
    
    // update current obstacle list
    auto cp = currObstacles_.find(id);
    if (cp == currObstacles_.end()) {
      currObstacles_.emplace(id, obstacle);
    }
    else {
      cp->second.swap(obstacle);
    }

    // refresh grid_ with new obstacle position
    auto lastp = lastObstacles_.find(id);
    auto currObs = currObstacles_.find(id)->second;
    if (lastp == lastObstacles_.end()) {
      for (auto i : currObs) {
        grid_->data[i.first] = 1;
      }
    }
    else {
      auto lastObs = lastp->second;
      for (auto j : lastObs) {
        grid_->data[j.first] = 0;
      }
      for (auto i : currObs) {
        grid_->data[i.first] = 1;
      }
    }

    // update last obstacle list
    auto lp = lastObstacles_.find(id);
    cp = currObstacles_.find(id);
    if (lp == lastObstacles_.end()) {
      lastObstacles_.emplace(id, cp->second);
    }
    else {
      lp->second = cp->second;
    }

    // automata
    Action action = Action::obstacleSetting;
    outputAutomata(action);
  }

  void Interface::obsProjection(const std::array<double, 4>& info,
                                std::unordered_map<int, bool>& obstacle) {
    const float cSize = grid_->info.resolution;
    // bounding box size length/width
		/// [unit: collision cells] -- The bounding box size length and width to precompute all possible headings
    double length = 1.2, width = 0.9;

    struct point {
      double x;
      double y;
    };

    // ______________________
    // VARIABLES FOR ROTATION
    //center of the rectangle
    point c;
    // points of the rectangle
    point p[4];
    point nP[4];

    // ____________________________
    // VARIABLES FOR GRID TRAVERSAL
    // vector for grid traversal
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;

    // set the starting angle to zero;
    double theta = info[3];
    // set points of rectangle
    c.x = info[0] / cSize;
    c.y = info[1] / cSize;

    p[0].x = - length / 2 / cSize;
    p[0].y = - width / 2 / cSize;

    p[1].x = - length / 2 / cSize;
    p[1].y = width / 2 / cSize;

    p[2].x = length / 2 / cSize;
    p[2].y = width / 2 / cSize;

    p[3].x = length / 2 / cSize;
    p[3].y = - width / 2 / cSize;

    // shape rotation
    for (int j = 0; j < 4; ++j) {
      // rotate and shift back
      nP[j].x = p[j].x * cos(theta) - p[j].y * sin(theta) + c.x;
      nP[j].y = p[j].x * sin(theta) + p[j].y * cos(theta) + c.y;
    }

    // cell traversal clockwise
    for (int k = 0; k < 4; ++k) {
      // create the vectors clockwise
      if (k < 3) {
        start = nP[k]; // [unit: collision cell]
        end = nP[k + 1];
      } else {
        start = nP[k];
        end = nP[0];
      }

      //set indexes
      X = (int)start.x;
      Y = (int)start.y;
      // std::pair(idx, value)
      obstacle.emplace(Y*grid_->info.width+X,true);
      t.x = end.x - start.x;
      t.y = end.y - start.y;
      stepX = t.x >= 0 ? 1 : -1;
      stepY = t.y >= 0 ? 1 : -1;

      // width and height normalized by t
      if (t.x != 0) {
        tDeltaX = 1.f / std::abs(t.x);
      } else {
        tDeltaX = std::numeric_limits<double>::max();
      }

      if (t.y != 0) {
        tDeltaY = 1.f / std::abs(t.y);
      } else {
        tDeltaY = std::numeric_limits<double>::max();
      }

      // set maximum traversal values
      if (stepX > 0) {
        tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
      } else {
        tMaxX = tDeltaX * (start.x - (long)start.x);
      }

      if (stepY > 0) {
        tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
      } else {
        tMaxY = tDeltaY * (start.y - (long)start.y);
      }

      while ((int)end.x != X || (int)end.y != Y) {
        // only increment x if the t length is smaller and the result will be closer to the goal
        if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
          tMaxX = tMaxX + tDeltaX;
          X = X + stepX;
          obstacle.emplace(Y*grid_->info.width+X,true);
          // only increment y if the t length is smaller and the result will be closer to the goal
        } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
          tMaxY = tMaxY + tDeltaY;
          Y = Y + stepY;
          obstacle.emplace(Y*grid_->info.width+X,true);
        } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
          if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
            X = X + stepX;
            obstacle.emplace(Y*grid_->info.width+X,true);
          } else {
            Y = Y + stepY;
            obstacle.emplace(Y*grid_->info.width+X,true);
          }
        } else {
          // this SHOULD NOT happen
          std::cout << "\n--->tie occured, please check for error in script\n";
          break;
        }
      }
    }
  }

  bool Interface::outputAutomata(Action action) {
    switch (automata_)
    {
    case initial:
      if (hasMap_ && isStartvalid_ && isGoalvalid_){
        automata_ = plan;
        setAllOutput();
        simulate(planningMap);
        automata_ = planned;
      }
      break;
    case plan:
      /* no action executed */
      /* this action should not really exist in this automata
         as it is a temporary value */
      break;
    case planned:
      if (action == obstacleSetting || 
          action == mapSetting || 
          action == goalSetting) {
        automata_ = initial;
        outputAutomata(action);
      } else if(action == startSetting) {
        automata_ = startReplan;
        outputAutomata(action);
      }
      break;
    case startReplan:
      setStartOutput();
      simulate(planningMap);
      automata_ = planned;
      break;
    case obsReplan:
      simulate(planningMap);
      automata_ = planned;
      break;
    default:
      std::cerr << "AUTOMATA ERROR" << std::endl;
      break;
    }
  }

  bool** Interface::makeBinMap() {
    bool **p = new bool*[grid_->info.width];
    for(size_t i = 0; i<grid_->info.width; ++i) {
      p[i] = new bool[grid_->info.height];
    }

    for (size_t i = 0; i<grid_->info.width; ++i) {
      for (size_t j = 0; j<grid_->info.height; ++j) {
        p[i][j] = grid_->data[j*grid_->info.width+i] ? true : false;
      }
    }

    return p;
  }

  bool Interface::setAllOutput() {
    // output to map object
    planningMap->setMap(grid_);
    configSpace->setGrid(grid_); // set the grid for configuration space
    configSpace->makeClsLookup();  // make up look up table in configuration space


    // output to planner
    // initialize using planning map resolution
    // TODO: change this cell size and angle size to a class
    auto start = std::make_shared<SE2State>(0.5,0.08726646);
    SE2State goal(carPlant_->planResolution,carPlant_->planAngleResolution);
    start->setX(start_.pose.pose.position.x);
    start->setY(start_.pose.pose.position.y);
    goal.setX(goal_.pose.position.x);
    goal.setY(goal_.pose.position.y);
    start->setT(Utils::normalizeHeadingRad(tf::getYaw(start_.pose.pose.orientation)));
    goal.setT(Utils::normalizeHeadingRad(tf::getYaw(goal_.pose.orientation)));
    // TODO: extract this planner into a specified class
    // Common::hRScurve d(*start,goal);
    
    // std::unique_ptr<Common::hRScurve> e(new Common::hRScurve(*start,goal));
    // auto c = std::make_unique<Common::hRScurve>(*start, goal);
    // HAstar b(start,goal,planningMap,configSpace);
    auto a = std::make_unique<HAstar>(start,goal,planningMap,configSpace);
    planner_.reset(a.release());

    return true;
  }

  void Interface::simulate(std::shared_ptr<Map<SE2State>> pmap) {
    // retrieve start point and goal point
    auto nStart = planner_->getStart();
    auto nGoal = planner_->getGoal();
    
    visualizer_.clear();
    smoother_.clearPath();
    path_.clear();
    smoothedPath_.clear();

    auto t0 = ros::Time::now();
    auto nSolution = planner_->solve(); auto t1 = ros::Time::now();
    smoother_.tracePath(nSolution);
    path_.updatePath(smoother_.getPath());
    smoother_.smoothPath(pmap->info_.width, pmap->info_.height); 
    auto t4 = ros::Time::now();
    smoothedPath_.updatePath(smoother_.getPath());
    ros::Duration d1(t1 - t0);
    ros::Duration d2(t4 - t1);

    std::cout << "TIME in ms:" << d1*1000 << '\t' << d2*1000 << std::endl;

    path_.publishPath();
    path_.publishPathNodes();
    path_.publishPathVehicles();
    smoothedPath_.publishPath();
    smoothedPath_.publishPathNodes();
    smoothedPath_.publishPathVehicles();
    visualizer_.publishNode3DCosts(pmap);

    smoother_.splinePub();
    assignTask();
  }

  void Interface::assignTask() {
    auto path = smoother_.returnTrajSet();
    auto direction = smoother_.returnDirectionSet();
    std_msgs::String temp;
    
    if (path.size() > 0){
      temp.data = Utils::writePath2String(direction, path);
      pubPath_.publish(temp);
    }
  }

  void Interface::setStartOutput(){
    planningMap->resetSS();
    // TODO: due to undefined reference to carPlant->planResolution
    auto start = std::make_shared<SE2State>(0.5,0.08726646);
    start->setX(start_.pose.pose.position.x);
    start->setY(start_.pose.pose.position.y);
    start->setT(Utils::normalizeHeadingRad(tf::getYaw(start_.pose.pose.orientation)));

    planner_->setStart(start);
  }

  void Interface::clearStartandGoal() {
    isStartvalid_ = false;
    isGoalvalid_ = false;
  }
} // namespace HybridAStar

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "a_star");
  auto interface(std::make_unique<HybridAStar::Interface>());
  ros::spin();
  return 0;
}
