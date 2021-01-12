#pragma once 

#include <unordered_map>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include "../common/map/map.h"
#include "../common/statespace/GridState.h"
#include "../common/statespace/SE2State.h"
#include "../common/collision/clsDetection.h"
#include "../multibody/SingleForkLiftPlant.h"
#include "geometry/hybridastar/HAstar.h"
#include "aux/visualize.h"
#include "geometry/path.h"
//#include "geometry/dynamicvoronoi.h"
#include "math/bspline.h"

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
    ros::NodeHandle n_; ///< ros node handle
    ros::Publisher pubPath_;  ///< path publishe, publish path to control center
    ros::ServiceClient posClient_;  ///< position client, communicate with control center, getting goal pos and current agv pos
    ros::Subscriber subStart_;  ///< start point subscriber, communicate with rviz
    ros::Subscriber subGoal_; ///< goal point subscriber, communicate with rviz
    ros::Subscriber subMap_;  ///< map subscriber, communicate with map server of ros
    ros::Subscriber subObs_;  ///< obstacle subscriber, temporarily not be used
    ros::Publisher tempSubscriber_; ///< publisher used to communicate with rviz, advertise goal pose received from sensor
    bool isStartvalid_; ///< flag
    bool isGoalvalid_;  ///< goal flag
    bool hasMap_; ///< map flag
    enum CurrentState{initial, plan, planned, startReplan, obsReplan}automata_; ///< planner states
    enum Action{noAct,mapSetting,startSetting,goalSetting,obstacleSetting}; ///< actions which can be excuted in planner automata

    // raw datas from outside
    geometry_msgs::PoseWithCovarianceStamped start_;  ///< start point in planning
    geometry_msgs::PoseStamped goal_; ///< goal point in planning
    nav_msgs::OccupancyGrid::Ptr grid_; ///< grid map from map server
    std::shared_ptr<Map<SE2State>> planningMap; ///< planning map pointer
    std::shared_ptr<CollisionDetection> configSpace;   ///< configuration space
    std::unique_ptr<HAstar> planner_; ///< Hybrid A Star Planner
    Visualize visualizer_;
    Path path_; ///< path which is not smoothed
    Path smoothedPath_ = Path(true);///< The path smoothed and ready for the controller
    BSpline smoother_;///< b spline smoother
    std::unique_ptr<Multibody::SingleForkLiftPlant> carPlant_;  ///< car configuration parameters
    std::unordered_map<int, std::unordered_map<int, bool>> lastObstacles_;  ///< not in use
    std::unordered_map<int, std::unordered_map<int, bool>> currObstacles_;  ///< not in use
  public:
    explicit Interface();
    ~Interface() = default;

    Interface(const Interface &) = delete;
    Interface &operator=(const Interface&) = delete;

    /**
     * @brief Set the goal point for planning. triggered by start point subscriber.
     *         When there is a new start being published by rviz topic, 
     *        the subscriber will fetch that start message and trigger this method
     * 
     * @param  start            the start message published by rviz topic
     */
    void makeStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);

    /**
     * @brief Set the start point for planning. Triggered by goal point subscriber. When there is a 
     *        new goal being published by rviz topic, 
     *        the subscriber will fetch that goal message and trigger this method
     * 
     * @param  goal             the goal message published by rviz topic
     */
    void makeGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);

    /**
     * @brief Set the Map object. Raw grid/pixel map for the whole planning. Triggered by map subscriber.
     *        When there is a new map published by rviz topic, the subscriber will fetch it and trigger
     *        this method
     * 
     * @param  map              the map published by map server
     */
    void setMap(const nav_msgs::OccupancyGrid::Ptr map);

    /**
     * @brief update obstacles. However, this method is not in use
     * 
     * @param pose 
     */
    void updateObs(const geometry_msgs::PoseStampedPtr& pose);

    /**
     * @brief project the obstacle posture into collision grid.(not in use)
     * 
     */
    void obsProjection(const std::array<double, 4>&,
                      std::unordered_map<int, bool>&);

    /**
     * @brief clear \p isStartvalid_ and \p isGoalvalid_ two flags. With any one of these two flags turned to
     *        be \b false , planning cannot proceed
     * 
     */
    void clearStartandGoal();

    /**
     * @brief planner automata. controls how the planner react according to \p Action input
     * 
     * @param \p Action       Actions input to the automata
     * @return true if transition succeed
     * @return false if transition fail
     */
    bool outputAutomata(Action);

    /**
     * @brief according to \p grid_, \p start_ and \p goal_, create planner
     * 
     * @return true 
     * @return false 
     */
    bool setAllOutput();

    /**
     * @brief take new start point and set it to planner
     * 
     */
    void setStartOutput();

    /**
     * @brief run planner
     * 
     */
    void simulate(std::shared_ptr<Map<SE2State>>);

    /**
     * @brief based on \p pubPath_ , send the path to control center
     * 
     */
    void assignTask();

    /**
     * @brief make binary map according to \p grid_
     * 
     * @return true 
     * @return false 
     */
    bool** makeBinMap();
  };
}