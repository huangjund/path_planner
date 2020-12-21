#ifndef _HYBRID_A_STAR_STATE_H_
#define _HYBRID_A_STAR_STATE_H_

#include <memory>
#include <vector>
#include <utility>

#include <boost/concept_check.hpp>

#include "utils/PtrWrapper.h"

namespace HybridAStar {
namespace Common {
  CLASS_SHARED(State);
  CLASS_SHARED(CompoundState);
  
  class State {
   public:
    template <class T>
    const T *as() const;

    template <class T>
    T *as();
    
   protected:
    State() = default;
    virtual ~State() = default;

   private:
    // disable copy-constructor
    State(const State &) = delete;
    // disable copy operator
    const State &operator=(const State &) = delete;
  };

  class CompoundState : public State {
   public:
    CompoundState() = default;
    virtual ~CompoundState() override = default;

    template <typename T>
    T* as(unsigned int i);

    template <typename T>
    const T* as(unsigned int i) const;

    StatePtr& operator[](unsigned int i) { return components[i]; }

    const StatePtr& operator[](unsigned int i) const { return components[i]; }

    void addState(const StatePtr&& state);
   protected:
    std::vector<StatePtr> components;
  };
}
}

#endif