#pragma once

#include <boost/concept_check.hpp>

namespace HybridAStar {
namespace Common {
  class State {
  public:
    template <class T>
    const T *as() const {
      // make sure the type we are allocating is indeed a state class
      BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

      return static_cast<const T*>(this);
    }

    template <class T>
    T *as() {
      // make sure the type we are allocating is indeed a state class
      BOOST_CONCEPT_ASSERT((boost::Convertible<T*, State*>));

      return static_cast<const T*>(this);
    }
    
  protected:
    State() = default;
    virtual ~State() = default;

  private:
    // disable copy-constructor
    State(const State &) = delete;
    // disable copy operator
    const State &operator=(const State &) = delete;
  };
}
}