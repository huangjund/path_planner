#ifndef _HYBRID_A_STAR_PLANNER_H
#define _HYBRID_A_STAR_PLANNER_H

#include <utility>

#include "utils/PtrWrapper.h"
#include "common/parameters/parameters.h"
#include "common/statespace/SE2State.h"
#include "common/PlannerTerminationCondition.h"

#include <boost/concept_check.hpp>

namespace HybridAStar {
namespace Geometry {
  CLASS_SHARED(Planner);
  CLASS_UNIQUE(Planner);
  CLASS_SHARED(CompoundPlanner);
  CLASS_UNIQUE(CompoundPlanner);

  class Planner {
   protected:
    Common::StatePtr start_;
    Common::StatePtr goal_;
    
   public:
    Planner() = default;
    explicit Planner(const Common::StatePtr&,
                     const Common::StatePtr&);
    virtual ~Planner() = default;
  
    Planner(const Planner &) = delete;
    Planner &operator=(const Planner &) = delete;

    Common::StatePtr& getMutableStart() { return start_; }
    const Common::StatePtr& getStart() const { return start_; }
    Common::StatePtr& getMutableGoal() { return goal_; }
    const Common::StatePtr& getGoal() const { return goal_; }
    
    /*SET serial method will be writen according to needs*/
    void setStartGoal(const Common::StatePtr&, const Common::StatePtr&);

    virtual void solve(const Common::PlannerTerminationCondition& ptc) = 0;
  };

  class CompoundPlanner : public Planner {
    public:
      CompoundPlanner() = default;
      explicit CompoundPlanner(const Common::StatePtr&,
                               const Common::StatePtr&);
      virtual ~CompoundPlanner() override = default;
      /**
       * @brief cast the instance to a desired type
       * 
       * @tparam T desired type
       * @param i the i th instance in components
       */
      template<typename T>
      T* as(unsigned int i);

      /**
       * @brief cast the instance to a desired type
       * 
       * @tparam T desired type
       * @param i the i th instance in components
       */
      template<typename T>
      const T* as(unsigned int i) const;

      /**
       * @brief get the i th planner through operator []
       * 
       * @param i 
       * @return PlannerPtr& 
       */
      PlannerPtr& operator[](unsigned int i) { return components[i]; }

      /**
       * @brief get the i th planner through operator []
       * 
       * @param i 
       * @return const PlannerPtr& 
       */
      const PlannerPtr& operator[](unsigned int i) const { return components[i]; }

      /**
       * @brief add a sub-planner to main Planner
       * 
       */
      void addPlanner(const PlannerUnique&&);
    protected:
      std::vector<PlannerUnique> components;
  };
} // namespace Geometry
} // namespace HybridAStar

#endif