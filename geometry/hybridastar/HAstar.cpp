#include "HAstar.h"

#include <boost/heap/binomial_heap.hpp>
#include "utils/Helper.h"


namespace HybridAStar{
namespace Geometry{
  struct CompareNodes {
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const SE2State* lhs, const SE2State* rhs) const {
      return lhs->getC() > rhs->getC();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const GridState* lhs, const GridState* rhs) const {
      return lhs->getC() > rhs->getC();
    }
  };

  HAstar::HAstar(SE2State &start,SE2State &goal,unique_ptr<Map<GridState>> &cmap,
      unique_ptr<Map<SE2State>> &pmap,CollisionDetection &configSpace): Planner(start,goal),
      start_(start),goal_(goal),configSpace_(configSpace), rsPlanner_(std::make_unique<hRScurve>(start_,goal_)),
      rrtxPlanner_(std::make_unique<hRRTx>(start_,goal_,cmap->info_.width,cmap->info_.height,configSpace)){

  }

  void HAstar::updateHeuristic(SE2State &start,SE2State &goal) {
    rsPlanner_->setStartGoal(start,goal);
    auto rsCost = rsPlanner_->getDistance();

    rrtxPlanner_->setStartGoal(start,goal);
    auto rrtxCost = rrtxPlanner_->getDistance();

    start.setH(std::max(rrtxCost,rsCost));
  }

  SE2State* HAstar::dubinsShot(SE2State& start, const SE2State& goal) {
    // start
    double q0[] = { start.getX(), start.getY(), start.getT() };
    // goal
    double q1[] = { goal.getX(), goal.getY(), goal.getT() };
    // initialize the path
    DubinsPath path;
    // calculate the path
    dubins_init(q0, q1, carPlant_->rad_, &path);

    int i = 0;
    float x = 0.f;
    float length = dubins_path_length(&path);

    SE2State* dubinsNodes = new SE2State[(int)(length / carPlant_->dubinsStepSize_) + 1]();
    
    while (x <  length) {
      double q[3];
      dubins_path_sample(&path, x, q);
      dubinsNodes[i].setX(q[0]);
      dubinsNodes[i].setY(q[1]);
      dubinsNodes[i].setT(Utils::normalizeHeadingRad(q[2]));

      // collision check
      if (configSpace_.isTraversable(&dubinsNodes[i])) {

        // set the predecessor to the previous step
        if (i > 0) {
          dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
        } else {
          dubinsNodes[i].setPred(&start);
        }

        if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
          std::cout << "looping shot";
        }

        x += carPlant_->dubinsStepSize_;
        i++;
      } else {
        // collided, discarding the path
        delete [] dubinsNodes;
        return nullptr;
      }
    }

    // // return &dubinsNodes[i - 1];
    // ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    // State* rsStart = (State*)reedsSheppPath.allocState();
    // State* rsEnd = (State*)reedsSheppPath.allocState();
    // rsStart->setXY(start.getX(), start.getY());
    // rsStart->setYaw(start.getT());
    // rsEnd->setXY(goal.getX(), goal.getY());
    // rsEnd->setYaw(goal.getT());

    
  }

  // solve for the whole path
  SE2State* HAstar::solve(){
    auto pWidth = pMap_->info_.width;
    auto pHeight = pMap_->info_.height;
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
    typedef boost::heap::binomial_heap<SE2State*,
            boost::heap::compare<CompareNodes>
            > priorityQueue;
    priorityQueue O;

    // update h value
    updateHeuristic(start_, goal_);
    // mark start_ as open
    start_.open();
    // push on priority queue open list
    O.push(&start_);
    iPred = start_.setIdx(pMap_->info_.width, pMap_->info_.height);
    pMap_->statespace[iPred] = start_;

    // NODE POINTER
    SE2State* nPred;
    SE2State* nSucc;

    // float max = 0.f;

    // continue until O empty
    while (!O.empty()) {
      // pop node with lowest cost from priority queue
      nPred = O.top();
      // set index
      iPred = nPred->setIdx(pMap_->info_.width, pMap_->info_.height);
      iterations++;

      // RViz visualization
      try
      {
        visualization.publishNode3DPoses(*nPred);
        visualization.publishNode3DPose(*nPred);
        d.sleep();
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
      

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
          if (nPred->isInRange(goal_) && nPred->getPrim() < 3) {
            nSucc = dubinsShot(*nPred, goal_);

            if (nSucc != nullptr && *nSucc == goal_) {
              return nSucc;
            }
          }

          // ______________________________
          // SEARCH WITH FORWARD SIMULATION
          for (int i = 0; i < dir; i++) {
            // create possible successor
            nSucc = nPred->createSuccessor(i);
            // set index of the successor
            iSucc = nSucc->setIdx(pMap_->info_.width, pMap_->info_.height);

            // ensure successor is on grid and traversable / resolution:[meters/cell]
            if (nSucc->isOnGrid(pWidth*pMap_->info_.resolution/cMap_->info_.resolution,
              pHeight*pMap_->info_.resolution/cMap_->info_.resolution) && configSpace_.isTraversable(nSucc)) {

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
                    delete nSucc;
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
                  O.push(&pMap_->statespace[iSucc]);
                  delete nSucc;
                } else { delete nSucc; }
              } else { delete nSucc; }
            } else { delete nSucc; }
          }
        }
      }
    }
    std::cout << "iterations:" << iterations << std::endl;
    if (O.empty()) {
      return nullptr;
    }

    return nullptr;
  }
} // namespace Geometry
} // namespace HybridAStar
