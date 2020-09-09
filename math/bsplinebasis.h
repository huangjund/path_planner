#ifndef _HYBRID_A_STAR_B_SPLINE_BASIS_H
#define _HYBRID_A_STAR_B_SPLINE_BASIS_H

#include <vector>
#include <utility>

namespace HybridAStar
{
  class bsplinebasis
  {
  private:
    std::pair<int,int> subscript;
    std::vector<double> ctrlPoints;
  public:
    bsplinebasis(int i, int k, std::vector<double>& ctrlp);

  };  // class bsplinebasis
} // namespace HybridAStar


#endif