#include "hAStar.h"

namespace HybridAStar
{
namespace Common
{
  struct CompareNodes 
  {
    bool operator()(const std::shared_ptr<GridState> lhs, const std::shared_ptr<GridState> rhs) const {
      return lhs->getC() > rhs->getC();
    }
  };
  

  hAStar::hAStar(const std::shared_ptr<GridState>& start, const GridState& goal,std::shared_ptr<CollisionDetection>& config):
    start_(start), goal_(goal), config_(config), pMap_(std::unique_ptr<Map<GridState>>()){}

  void hAStar::setGoal(const GridState& goal){
    goal_ = goal;
  }

  void hAStar::setStart(const std::shared_ptr<GridState>& start) {
    start_ = start;
  }

  void hAStar::setStartGoal(const std::shared_ptr<GridState>& start, const GridState& goal) {
    start_ = start;
    goal_ = goal;
  }

  double hAStar::getDistance() {
    int iPred, iSucc;
    float newG;
    int pwidth = pMap_->info_.width*pMap_->info_.resolution/pMap_->info_.planResolution;
    int pheight = pMap_->info_.height*pMap_->info_.resolution/pMap_->info_.planResolution;
    int dir = 8;

    // reset the open and closed list
    for(auto i = pMap_->statespace.begin(); i<pMap_->statespace.end(); ++i ){
      (*i).reset();
    }

    boost::heap::binomial_heap<std::shared_ptr<GridState>,
                boost::heap::compare<CompareNodes>> O;

    start_->updateH(goal_);
    start_->open();
    O.push(start_);

    iPred = start_->setIdx(pwidth);
    pMap_->statespace[iPred] = *start_;

    std::shared_ptr<GridState> nPred;
    std::shared_ptr<GridState> nSucc;

    while(!O.empty()) {
      nPred = O.top();
      iPred = nPred->setIdx(pwidth);

      if(pMap_->statespace[iPred].isClosed()) {
        O.pop();
        continue;
      }

      else if(pMap_->statespace[iPred].isOpen()) {
        pMap_->statespace[iPred].close();
        O.pop();

        if(*nPred == goal_) {
          return nPred->getG();
        }

        else {
          for (int i = 0; i<dir; ++i) {
            nSucc.reset(nPred->createSuccessor(i,nPred));
            iSucc = nSucc->setIdx(pwidth);

            if (nSucc->isOnGrid(pwidth,pheight) && config_->isTraversable(nSucc.get()) &&
                !pMap_->statespace[iSucc].isClosed()) {
              nSucc->updateG();
              newG = nSucc->getG();
              if (!pMap_->statespace[iSucc].isOpen() || newG < pMap_->statespace[iSucc].getG()) {
                nSucc->updateH(goal_);
                nSucc->open();
                pMap_->statespace[iSucc] = *nSucc;
                O.push(nSucc);
              }
            }
          }
        }
      }
    }

    std::cerr << "ERROR: no A* path found" << std::endl;
    return std::numeric_limits<double>::min();
  }

  void hAStar::setRawmap(const nav_msgs::OccupancyGrid::Ptr& map) {
    pMap_->setMap(map);
  }

  void hAStar::setSS() {
    pMap_->setSS();
  }

  std::unique_ptr<Map<GridState>>& hAStar::returnMap() {
    return pMap_;
  }
} // namespace Common
  
} // namespace HybridAstar