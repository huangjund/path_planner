#include "astar.h"

namespace HybridAStar {
namespace Geometry {
AStar::AStar(const Common::StatePtr& start, 
             const Common::StatePtr& goal,
             const Common::CollisionDetectionPtr& config) :
             Planner(start, goal), config_(config) {
}

void AStar::setMap(const nav_msgs::OccupancyGrid::Ptr& map) {
  pMap_->setMap(map);
}

std::unique_ptr<Common::PlanningMap<Common::GridState>>&
      AStar::returnMap() {
  return pMap_;
}

void AStar::setSS(unsigned int count) {
  int pwidth = pMap_->getWid()*pMap_->colResolution_/PlanningMapConst::cellSize;
  int pheight = pMap_->getHeight()*pMap_->colResolution_/PlanningMapConst::cellSize;

  pMap_->setSS(pwidth*pheight);
}

void solve(const Common::PlannerTerminationCondition& ptc) {
  int iPred, iSucc;
  float newG;
  const int pwidth = pMap_->getWid()*pMap_->colResolution_/Common::PlanningMapConst::cellSize;
  const int pheight = pMap_->getHeight()*pMap_->colResolution_/Common::PlanningMapConst::cellSize;
  int dir = Common::aStarMotionPrimitives::dir;

  std::vector<Common::GridStatePtr> statespace(pwidth*pheight);

  using binomial_heap = boost::heap::binomial_heap<Common::GridStatePtr,
              boost::heap::compare<CompareNodes>>;
  binomial_heap O;
  std::vector<binomial_heap::handle_type> handler(pwidth*pheight);

  start_->updateH(*goal_);
  start_->open();
  O.push(start_);

  iPred = start_->setIdx(pwidth);
  statespace[iPred] = start_;

  Common::GridStatePtr nPred;
  Common::GridStatePtr nSucc;

  while(!O.empty()) {
    nPred = O.top();
    iPred = nPred->setIdx(pwidth);

    if(statespace[iPred]->isClosed()) {
      O.pop();
      continue;
    }

    else if(statespace[iPred]->isOpen()) {
      statespace[iPred]->close();
      O.pop();

      if(*nPred == *goal_) {
        goal_ = std::move(nPred);
        return;
      }

      else {
        for (int i = 0; i<dir; ++i) {
          nSucc.reset(nPred->createSuccessor(i,nPred));
          iSucc = nSucc->setIdx(pwidth);

          if (nSucc->isOnGrid(pwidth,pheight) && config_->isTraversable(nSucc.get()) &&
              (statespace[iSucc] == NULL || !statespace[iSucc]->isClosed())) {
            nSucc->updateG();
            newG = nSucc->getG();
            // if this point has not explored yet or g is greater
            if (statespace[iSucc] == NULL || !statespace[iSucc]->isOpen()) {
              nSucc->updateH(goal_);
              nSucc->open();
              statespace[iSucc] = nSucc;
              handler[iSucc] = O.push(nSucc);
            }
            else if(newG < statespace[iSucc]->getG()) {
              statespace[iSucc]->setG(newG);
              statespace[iSucc]->setPred(nSucc->getPred());
              O.update(handler[iSucc]);
            }
          }
        }
      }
    }
  }

  std::cerr << "ERROR: no A* path found" << std::endl;
  return;
}
} // namespace Geometry
} // namespace HybridAStar