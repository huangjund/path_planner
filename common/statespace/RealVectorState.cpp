#include "RealVectorState.h"

namespace HybridAStar {
namespace Common {

template <typename T>
RealVectorState<T>::RealVectorState(unsigned int dim) : dimension_(dim), values_(nullptr) {
  allocState();
}

template <typename T>
RealVectorState<T>::~RealVectorState() {
  freeState();
}

template <typename T>
State* RealVectorState<T>::allocState() {
  values_ = new T[dimension_];
  return this;
}

template <typename T>
void RealVectorState<T>::freeState() {
  delete[] values_;
}

template <typename T>
RealVectorState<T>::RealVectorState(const RealVectorState<T>& state) : 
                                dimension_(state.dimension_){
  if (values_ == nullptr) 
    allocState();

  for (auto i = 0; i < dimension_; ++i) 
    values_[i] = state[i];
}

template <typename T>
RealVectorState<T>& RealVectorState<T>::operator=(const RealVectorState<T>& rhs) {
  dimension_ = rhs.dimension_;
  if (values_ == nullptr) allocState();
  for (auto i = 0; i < dimension_; ++i) 
    values_[i] = rhs[i];
  return *this;
}

} // namespace Common
} // namespace HybridAStar