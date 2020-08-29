#include "HAstar.h"

#include <cassert>
#include <boost/heap/binomial_heap.hpp>
#include "utils/Helper.h"


namespace HybridAStar{
namespace Geometry{
  struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const std::shared_ptr<SE2State> lhs, const std::shared_ptr<SE2State> rhs) const {
      return lhs->getC() > rhs->getC();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const std::shared_ptr<GridState> lhs, const std::shared_ptr<GridState> rhs) const {
      return lhs->getC() > rhs->getC();
    }
  };

  HAstar::HAstar(std::shared_ptr<SE2State> start,SE2State &goal,shared_ptr<Map<SE2State>> pmap,CollisionDetection &configSpace): 
                Planner(*start,goal),start_(start),goal_(goal),configSpace_(&configSpace), 
                rsPlanner_(std::make_unique<hRScurve>(*start_,goal_)),
      rrtxPlanner_(std::make_unique<hRRTx>(*start_,goal_,
      pmap->info_.width*pmap->info_.resolution, pmap->info_.height*pmap->info_.resolution,configSpace)),
      pMap_(pmap){}

  void HAstar::updateHeuristic(SE2State &start,SE2State &goal) {
    rsPlanner_->setStartGoal(start,goal);
    auto rsCost = rsPlanner_->getDistance();

    rrtxPlanner_->setStartGoal(start,goal);
    auto rrtxCost = rrtxPlanner_->getDistance();

    start.setH(std::max(rrtxCost,rsCost));
  }

#ifdef DUBINS_H
  std::shared_ptr<SE2State> HAstar::dubinsShot(
    std::shared_ptr<SE2State> start, const SE2State& goal) {
    // start
    double q0[] = { start->getX(), start->getY(), start->getT() };
    // goal
    double q1[] = { goal.getX(), goal.getY(), goal.getT() };
    // initialize the path
    DubinsPath path;
    // calculate the path
    dubins_init(q0, q1, carPlant_->rad_, &path);

    float x = carPlant_->dubinsStepSize_;
    float length = dubins_path_length(&path);
    int i = 0;

    // TODO: this construction should not related to the default constructor
    // TODO: the angle resolution should be synthesized to a class
    auto dubinsNodes = std::vector<std::shared_ptr<SE2State>>(
      (int)(length / carPlant_->dubinsStepSize_) + 2);
      // std::make_shared<SE2State>(pMap_->info_.planResolution, 0.087266)
    
    while (x <  length) {
      // TODO: this angle resolution should be synthesized to a class
      auto temp = std::make_shared<SE2State>(pMap_->info_.planResolution, 0.087266);
      double q[3];
      dubins_path_sample(&path, x, q);
      temp->setX(q[0]);
      temp->setY(q[1]);
      temp->setT(Utils::normalizeHeadingRad(q[2]));
      dubinsNodes[i] = temp;

      // collision check
      if (configSpace_->isTraversable(dubinsNodes[i].get())) {

        // set the predecessor to the previous step
        if (i > 0) {
          dubinsNodes[i]->setPred(dubinsNodes[i-1]);
        } else {
          dubinsNodes[i]->setPred(start);
        }

        if ((dubinsNodes[i]) == dubinsNodes[i]->getPred()) {
          std::cout << "looping shot";
        }

        x += carPlant_->dubinsStepSize_;
        i++;
      } else {
        // collided, discarding the path
        std::shared_ptr<SE2State> temp;
        return temp;
      }
    }

    // TODO: change to a more delicated loop, which also include is Traversable judging
    auto temp = std::make_shared<SE2State>(pMap_->info_.planResolution, 0.087266);
    temp->setX(q1[0]);
    temp->setY(q1[1]);
    temp->setT(Utils::normalizeHeadingRad(q1[2]));
    temp->setPred(dubinsNodes[i-1]);
    dubinsNodes[i] = temp;

    return dubinsNodes[i];
  }
#endif

#ifdef _HYBRIDASTAR_REEDSSHEPPPATH_H
  std::shared_ptr<SE2State> HAstar::ReedsShepp(std::shared_ptr<SE2State> start, const SE2State &goal) {
    std::vector<double> bounds{0,(pMap_->info_.width*pMap_->info_.resolution),
                                0,(pMap_->info_.width*pMap_->info_.resolution)};
    auto startNode = std::make_shared<Node3d>(start->getX(),start->getY(),start->getT(),pMap_->info_.planResolution,0.08726646,bounds);
    auto goalNode = std::make_shared<Node3d>(goal.getX(),goal.getY(),goal.getT(),pMap_->info_.planResolution,0.08726646,bounds);

    // TODO:change the last 0.5 to a carplant related value
    // should carplant class be an abstract base class and other class inherit from this class?
    auto reed_shepp_generator = std::make_shared<ReedShepp>(1/carPlant_->rad_,0.5);
    ReedSheppPath optimal_path;

    if (reed_shepp_generator->ShortestRSP(startNode, goalNode,
                                           optimal_path) == false) {
      std::cout << "RS path generation failed!" << std::endl;
    }

    auto length = optimal_path.total_length;
    // TODO : change to a smart pointer
    auto RSNodes = std::vector<std::shared_ptr<SE2State>>(
      (int)(length / carPlant_->reedsheppStepSize_) + 3);

    int i;
    for (i = 0; i < optimal_path.x.size(); ++i) {
      // TODO: this angle resolution should be synthesized to a class
      auto temp = std::make_shared<SE2State>(pMap_->info_.planResolution, 0.087266);
      temp->setX(optimal_path.x[i]);
      temp->setY(optimal_path.y[i]);
      temp->setT(Utils::normalizeHeadingRad(optimal_path.phi[i]));
      RSNodes[i] = temp;

      // collision check
      if (configSpace_->isTraversable(RSNodes[i].get())) {

        // set the predecessor to the previous step
        if (i > 0) {
          RSNodes[i]->setPred(RSNodes[i - 1]);
        } else {
          RSNodes[i]->setPred(start);
        }

        if (RSNodes[i] == RSNodes[i]->getPred()) {
          std::cout << "looping shot";
        }

      } else {
        // collided, discarding the path
        std::shared_ptr<SE2State> temp;
        return temp;
      }
    }
    return RSNodes[i-1];
  }
#endif

  SE2State &HAstar::getStart() {
    return *start_;
  }

  SE2State &HAstar::getGoal() {
    return goal_;
  }

  void HAstar::setStart(std::shared_ptr<SE2State> start) {
    assert(rrtxPlanner_->isTreeconstructed());
    start_ = start;
    rsPlanner_->setStart(*start);
    rrtxPlanner_->setGoal(*start); // the goal and start in rrtx is reversed
  }

  // solve for the whole path
  std::shared_ptr<SE2State> HAstar::solve(){
    int pWidth = pMap_->info_.width*(pMap_->info_.resolution)/(pMap_->info_.planResolution);
    int pHeight = pMap_->info_.height*(pMap_->info_.resolution)/(pMap_->info_.planResolution);
    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;
    // Number of possible directions, 3 for forward driving and an additional 3 for reversing
    int dir = carPlant_->isReversable_ ? 6 : 3;
    // Number of iterations the algorithm has run for stopping based on Constants::iterations
    int iterations = 0;

    // VISUALIZATION DELAY
    ros::Duration d(0.003);

    // OPEN LIST AS BOOST IMPLEMENTATION
    typedef boost::heap::binomial_heap<std::shared_ptr<SE2State>,
            boost::heap::compare<CompareNodes>
            > priorityQueue;
    priorityQueue O;

    // update h value
    updateHeuristic(*start_, goal_);
    // mark start_ as open
    start_->open();
    // push on priority queue open list
    O.push(start_);
    iPred = start_->setIdx(pWidth, pHeight);
    pMap_->statespace[iPred] = *start_;

    // NODE POINTER
    std::shared_ptr<SE2State> nPred;
    std::shared_ptr<SE2State> nSucc;

    // float max = 0.f;

    // continue until O empty
    while (!O.empty()) {
      // pop node with lowest cost from priority queue
      nPred = O.top();
      // set index
      iPred = nPred->setIdx(pWidth, pHeight);
      iterations++;
      // TODO: enable this
      // // RViz visualization
      // try
      // {
      //   visualization.publishNode3DPoses(*nPred);
      //   visualization.publishNode3DPose(*nPred);
      //   d.sleep();
      // }
      // catch(const std::exception& e)
      // {
      //   std::cerr << e.what() << '\n';
      // }
      
      // _____________________________
      // LAZY DELETION of rewired node
      // if there exists a pointer this node has already been expanded
      if (pMap_->statespace[iPred].isClosed()) {
        // pop node from the open list and start_ with a fresh node
        O.pop();
        continue;
      }
      // _________________
      // EXPANSION OF NODE
      else if (pMap_->statespace[iPred].isOpen()) {
        // add node to closed list
        pMap_->statespace[iPred].close();
        // remove node from open list
        O.pop();

        // _________
        // GOAL TEST
        if (*nPred == goal_ || iterations > defaultIter) {
          // DEBUG
          return nPred;
        }
        
        // ____________________
        // CONTINUE WITH SEARCH
        else {
          // _______________________
          // SEARCH WITH DUBINS SHOT
          // TODO:when changed to RS curve, the primitives < 3 needs to be removed
          // the isInRange function needs to be more scientific
          if (nPred->isInRange(goal_) && nPred->getPrim() < 3) {
            //nSucc = dubinsShot(nPred, goal_);
            nSucc = ReedsShepp(nPred, goal_);
            if (nSucc != nullptr && *nSucc == goal_) {
              std::cout << "iterations:" << iterations << std::endl;
              
              return nSucc;
            }
          }

          // ______________________________
          // SEARCH WITH FORWARD SIMULATION
          for (int i = 0; i < dir; i++) {
            // create possible successor
            nSucc.reset(nPred->createSuccessor(i,nPred));
            // set index of the successor
            iSucc = nSucc->setIdx(pWidth, pHeight);

            // ensure successor is on grid and traversable / resolution:[meters/cell]
            if (nSucc->isOnGrid(static_cast<int>(pWidth), static_cast<int>(pHeight)) && configSpace_->isTraversable(nSucc.get())) {

              // ensure successor is not on closed list or it has the same index as the predecessor
              if (!pMap_->statespace[iSucc].isClosed() || iPred == iSucc) {

                // calculate new G value
                nSucc->updateG();
                newG = nSucc->getG();

                // if successor not on open list or found a shorter way to the cell
                if (!pMap_->statespace[iSucc].isOpen() || newG < pMap_->statespace[iSucc].getG() || iPred == iSucc) {

                  // calculate H value
                  updateHeuristic(*nSucc, goal_);

                  // if the successor is in the same cell but the C value is larger
                  if (iPred == iSucc && nSucc->getC() > nPred->getC() + carPlant_->tieBreaker_) {
                    continue;
                  }
                  // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                  else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + carPlant_->tieBreaker_) {
                    nSucc->setPred(nPred->getPred());
                  }

                  if (nSucc->getPred() == nSucc) {
                    std::cout << "looping";
                  }

                  // put successor on open list
                  nSucc->open();
                  pMap_->statespace[iSucc] = *nSucc;
                  O.push(nSucc);
                }
              }
            }
          }
        }
      }
    }
    std::cout << "iterations:" << iterations << std::endl;
    std::shared_ptr<SE2State> temp;
    if (O.empty()) {
      return temp;
    }

    return temp;
  }
} // namespace Geometry
} // namespace HybridAStar
