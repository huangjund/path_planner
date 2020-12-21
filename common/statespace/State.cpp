#include "State.h"

namespace HybridAStar {
namespace Common {

template <class T>
const T* State::as() const {
  // make sure the type we are allocating is indeed a state class
  BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

  return static_cast<const T*>(this);
} 

template <class T>
T* State::as() {
  // make sure the type we are allocating is indeed a state class
  BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

  return static_cast<const T*>(this);
}

template <class T>
T* CompoundState::as(unsigned int i) {
  BOOST_CONCEPT_ASSERT((boost::Convertible<T*,State*>));

  return static_cast<T*>(component[i].get());
}

template <class T>
const T* CompoundState::as(unsigned int i) const {
  BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

  return static_cast<const T*>(component[i].get());
}

void CompoundState::addState(const StatePtr&& se2) {
  components.push_back(std::forward<StatePtr&&>(se2));
}
} // namespace Common
} // namespace HybridAStar