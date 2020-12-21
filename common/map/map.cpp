#include "map.h"

namespace HybridAStar {
namespace Common {
template <typename T>
Map<T>::Map(T length, T width) : width_(length), height_(width){}

template <typename T>
Map<T>::Map(const Map<T>& m) : width_(m.getWid()), height_(m.getHeight()) {}

template <typename T>
Map<T>& Map<T>::operator=(const Map<T>& m) {
  width_ = m.getWid();
  height_ = m.getHeight();
  return *this;
}


} // namespace Common
} // namespace HybridAStar