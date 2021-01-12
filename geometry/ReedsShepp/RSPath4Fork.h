#ifndef _HYBRIDASTAR_REEDSHEPP_PATH_FOR_FORK_H
#define _HYBRIDASTAR_REEDSHEPP_PATH_FOR_FORK_H

#include "ReedsSheppPath.h"

namespace HybridAStar
{
namespace Geometry
{
/**
 * @brief Reeds shepp planner for fork
 * 
 */
class RSPath4Fork : public ReedShepp
{
protected:
  virtual bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths) override;
  virtual bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths) override;
  virtual bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths) override;
  virtual bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths) override;
  virtual bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths) override;
  virtual bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths) override;
public:
  RSPath4Fork(const double max_kappa, const double step_size = 0.1);

  double ShortestRSPlength(const std::shared_ptr<Node3d> start_node,
                            const std::shared_ptr<Node3d> end_node);

  std::vector<ReedSheppPath> possiblePath(const std::shared_ptr<Node3d> start_node,
                                          const std::shared_ptr<Node3d> end_node);
                                        

};
} // namespace Geometry

} // namespace HybridAStar

#endif