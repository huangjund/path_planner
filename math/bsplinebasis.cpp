#include "bsplinebasis.h"

using namespace HybridAStar;

bsplinebasis::bsplinebasis(int i, int k, std::vector<double>& ctrlp)
  :subscript(i,k),ctrlPoints(ctrlp) {}

  