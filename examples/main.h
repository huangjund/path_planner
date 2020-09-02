#pragma once 

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "../common/map/map.h"
#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"
#include "../common/collision/clsDetection.h"
#include "../multibody/SingleForkLiftPlant.h"
#include "geometry/hybridastar/HAstar.h"
#include "aux/visualize.h"
#include "geometry/path.h"
#include "geometry/dynamicvoronoi.h"
#include "math/smoother.h"

namespace HybridAStar{
  using HybridAStar::Common::CollisionDetection;
  using HybridAStar::Common::GridState;
  using HybridAStar::Common::SE2State;
  using HybridAStar::Common::Map;
  using HybridAStar::Geometry::HAstar;
  using HybridAStar::Multibody::SingleForkLiftPlant;
  /**
   * @brief the interface class is a whole block that connect each planning components
   *        with each other
   */
  class Interface
  {
  private:
    ros::NodeHandle n_;
    ros::Subscriber subStart_;
    ros::Subscriber subGoal_;
    ros::Subscriber subMap_;
    bool isStartvalid_;
    bool isGoalvalid_;
    bool hasMap_;
    enum CurrentState{initial, plan, planned, replan}automata_;
    enum Action{noAct,mapSetting,startSetting,goalSetting};

    // raw datas from outside
    geometry_msgs::PoseWithCovarianceStamped start_;
    geometry_msgs::PoseStamped goal_;
    nav_msgs::OccupancyGrid::Ptr grid_;
    std::shared_ptr<Map<SE2State>> planningMap;
    std::shared_ptr<CollisionDetection> configSpace;
    std::unique_ptr<HAstar> planner_;
    Visualize visualizer_;
    Path path_;
    Path smoothedPath_ = Path(true);/// The path smoothed and ready for the controller
    Smoother smoother_;/// The smoother used for optimizing the path
    DynamicVoronoi voronoiDiagram_;
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;
  public:
    explicit Interface();
    ~Interface() = default;

    Interface(const Interface &) = delete;
    Interface &operator=(const Interface&) = delete;

    /**
     * @brief Set the goal point for planning. triggered by a subscriber. When there is a new start being published by rviz topic, 
     *        the subscriber will fetch that start message and trigger this method
     * 
     * @param  start            the start message published by rviz topic
     */
    void makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

    /**
     * @brief Set the start point for planning. Triggered by a subscriber. When there is a 
     *        new goal being published by rviz topic, 
     *        the subscriber will fetch that goal message and trigger this method
     * 
     * @param  goal             the goal message published by rviz topic
     */
    void makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

    /**
     * @brief Set the Map object. Raw grid/pixel map for the whole planning. Triggered by a subscriber.
     *        When there is a new map published by rviz topic, the subscriber will fetch it and trigger
     *        this method
     * 
     * @param  map              the map published by map server
     */
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);

    /**
     * @brief clear \p isStartvalid_ and \p isGoalvalid_ two flags. With any one of these two flags turned to
     *        be \b false , planning cannot proceed
     * 
     */
    void clearStartandGoal();

    /**
     * @brief 
     * 
     * @param \p Action       Actions input to the automata
     * @return true 
     * @return false 
     */
    bool outputAutomata(Action);
    bool setAllOutput();
    void setStartOutput();
    void simulate(std::shared_ptr<Map<SE2State>>);
  };
}