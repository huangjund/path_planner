#include "Planner.h"

namespace HybridAStar{
namespace Geometry{
  Planner::Planner(const Common::StatePtr& start,
                   const Common::StatePtr& goal):
    start_(start),goal_(goal){}

  CompoundPlanner::CompoundPlanner(const Common::StatePtr& start,
                                   const Common::StatePtr& goal):
                                  Planner(start,goal) {};

  void Planner::setStartGoal(const Common::StatePtr& start,
                             const Common::StatePtr& goal) {
    start_ = start;
    goal_ = goal;
  }

  template <typename T>
  T* CompoundPlanner::as(unsigned int i) {
    /** \brief make sure the planner can be converted to T*/
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));

    return static_cast<T*>(components[i].get());
  }

  template <typename T>
  const T* CompoundPlanner::as(unsigned int i ) const {
    /** \brief make sure the planner can be converted to T*/
    BOOST_CONCEPT_ASSERT((boost::Convertible<T*, Planner*>));

    return static_cast<const T*>(components[i].get);
  }

  void CompoundPlanner::addPlanner(const PlannerUnique&& planner) {
    components.push_back(std::forward<PlannerUnique&&>(planner));
  }
} // namespace Geometry
} // namespace HybridAStar
