#include "algorithm.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

float aStar(Node2D&, Node2D&, Node2D*, int, int, CollisionDetection&, Visualize&);
void updateH(PlanMapNode&, const PlanMapNode&, Node2D*, float*, int, int, CollisionDetection&, Visualize&);
PlanMapNode* dubinsShot(PlanMapNode&, const PlanMapNode&, CollisionDetection&);

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const PlanMapNode* lhs, const PlanMapNode* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
PlanMapNode* Algorithm::hybridAStar(PlanMapNode& start,
                               const PlanMapNode& goal,
                               PlanMapNode* planningMap,
                               Node2D* map2D,
                               const int planMapwidth,
                               const int planMapHeight,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // VISUALIZATION DELAY
  ros::Duration d(0.003);

  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<PlanMapNode*, boost::heap::compare<CompareNodes>> priorityQueue;
  priorityQueue O;

  // update h value
  updateH(start, goal, map2D, dubinsLookup, planMapwidth, planMapHeight, configurationSpace, visualization);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(planMapwidth, planMapHeight);
  planningMap[iPred] = start;

  // NODE POINTER
  PlanMapNode* nPred;
  PlanMapNode* nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(planMapwidth, planMapHeight);
    iterations++;

    // RViz 3D visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if this predecessor node has already been expanded
    if (planningMap[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (planningMap[iPred].isOpen()) {
      // add node to closed list
      planningMap[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      // if the start point is at the goal or iteration exceeds the limit
      if (*nPred == goal || iterations > Constants::iterations) {
        // DEBUG
        std::cout << "number of iterations: " << iterations << std::endl;
        return nPred;
      }
      else {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        // when nPred shoots the analytical path range, motion primitives from its predecessor is not about reverse
        // and we hope to use dubins curve method, then we have:
        // TODO: dubinsShot doesn't check for on grid limit
        if (Constants::dubinsShot && nPred->isInRange(goal) && nPred->getPrim() < 3) {
          nSucc = dubinsShot(*nPred, goal, configurationSpace);

          if (nSucc != nullptr && *nSucc == goal) {
            //DEBUG
            std::cout << "number of iterations: " << iterations << std::endl;
            return nSucc;
          }
        }

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        for (int i = 0; i < dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(planMapwidth, planMapHeight);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(planMapwidth, planMapHeight, Constants::collisionMapCellSize) &&
            configurationSpace.isTraversable(nSucc)) {

            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!planningMap[iSucc].isClosed() || iPred == iSucc) {

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!planningMap[iSucc].isOpen() || newG < planningMap[iSucc].getG() || iPred == iSucc) {

                // calculate H value
                updateH(*nSucc, goal, map2D, dubinsLookup, planMapwidth, planMapHeight, configurationSpace, visualization);

                // if the successor is in the same cell with its predecessor but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                planningMap[iSucc] = *nSucc;
                O.push(&planningMap[iSucc]);
                delete nSucc;
              } else { delete nSucc; }
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D& start,
            Node2D& goal,
            Node2D* map2D,
            int planMapWidth,
            int planMapHeight,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < planMapWidth*planMapHeight; ++i) {
    map2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(planMapWidth);
  map2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(planMapWidth);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (map2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (map2D[iPred].isOpen()) {
      // add node to closed list
      map2D[iPred].close();
      map2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(planMapWidth);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(planMapWidth, planMapHeight) &&  configurationSpace.isTraversable<Node2D>(nSucc) && !map2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!map2D[iSucc].isOpen() || newG < map2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              map2D[iSucc] = *nSucc;
              O.push(&map2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(PlanMapNode& start, 
            const PlanMapNode& goal, 
            Node2D* map2D, 
            float* dubinsLookup, 
            int planMapWidth, int planMapHeight, 
            CollisionDetection& configurationSpace, 
            Visualize& visualization) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use R&S curve
  if (Constants::reverse && !Constants::dubins) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  const auto startidx = (int)start.getry()*planMapWidth + (int)start.getrx();
  if (Constants::twoD && !map2D[startidx].isDiscovered()) {
    // create a 2d start node
    Node2D start2d(static_cast<int>(start.getrx()), static_cast<int>(start.getry()), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(static_cast<int>(goal.getrx()), static_cast<int>(goal.getry()), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    map2D[startidx].setG(aStar(goal2d, start2d, map2D, planMapWidth, planMapHeight, configurationSpace, visualization));
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getrx() - (long)start.getrx()) - (goal.getrx() - (long)goal.getrx()))*
                      ((start.getrx() - (long)start.getrx()) - (goal.getrx() - (long)goal.getrx()))+
                      ((start.getry() - (long)start.getry()) - (goal.getry() - (long)goal.getry()))*
                      ((start.getry() - (long)start.getry()) - (goal.getry() - (long)goal.getry())));
    twoDCost = map2D[startidx].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
}

//###################################################
//                                        DUBINS SHOT
//###################################################
PlanMapNode* dubinsShot(PlanMapNode& start,
                      const PlanMapNode& goal,
                      CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  std::vector<Node3D> dubinsNodes((int)(length / Constants::dubinsStepSize) + 1);

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable<Node3D>(&dubinsNodes[i])) {
      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setnodePred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setnodePred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getnodePred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      return nullptr;
    }
  }

  // set the dubins curve to planning map
  PlanMapNode* dubinsPlanMap = new PlanMapNode [i];
  int j = 0;
  for (auto &temp : dubinsNodes) {
    dubinsPlanMap[j].setrx(temp.getX()/PlanMapNode::planmapCellSize);
    dubinsPlanMap[j].setry(temp.getY()/PlanMapNode::planmapCellSize);
    dubinsPlanMap[j].setrt(temp.getT()/PlanMapNode::planmapAngleSize);
    if (j == 0) {
      dubinsPlanMap[j].setPred(&start);
    } else
    {
      dubinsPlanMap[j].setPred(&dubinsPlanMap[j-1]);
    }
  }
  
  return &dubinsPlanMap[i - 1];
}
