#ifndef _HYBRIDASTAR_REEDSHEPP_PATH_FOR_FORK_H
#define _HYBRIDASTAR_REEDSHEPP_PATH_FOR_FORK_H

#include "ReedsSheppPath.h"
#include "common/statespace/SE2State.h"

namespace HybridAStar
{
namespace Geometry
{
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
  RSPath4Fork(const double max_kappa,
              const double step_size);

  RSPath4Fork(const Common::SE2StatePtr&,
              const Common::SE2StatePtr&);

  RSPath4Fork(const Common::SE2StatePtr&,
              const Common::SE2StatePtr&,
              const double max_kappa, 
              const double step_size = 0.1);

  double ShortestRSPlength();

  std::vector<ReedSheppPath> possiblePath();
                                        

};
} // namespace Geometry

} // namespace HybridAStar

#endif