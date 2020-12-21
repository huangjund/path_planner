#ifndef _HYBRID_A_STAR_REAL_VECTOR_STATE_H_
#define _HYBRID_A_STAR_REAL_VECTOR_STATE_H_

#include "State.h"

namespace HybridAStar {
namespace Common {

/**
 * @brief 
 * 
 * @tparam T :double or int
 */
template <typename T>
class RealVectorState : public State {
 public:
  RealVectorState(unsigned int dim);
  virtual ~RealVectorState() override;

  RealVectorState(const RealVectorState<T>& other);
  RealVectorState& operator=(const RealVectorState<T>& rhs);

  T operator[](unsigned int i) const { return values_[i]; }
  T& operator[](unsigned int i) { return values_[i]; }

  int dimension_;
  T* values_;

 private:
  void freeState();
  State* allocState();
};

} // namespace HybridAStar
} // namespace Common

#endif