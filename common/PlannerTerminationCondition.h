#ifndef _HYBRID_A_STAR_PLANNER_TERMINATION_CONDITION_H
#define _HYBRID_A_STAR_PLANNER_TERMINATION_CONDITION_H

#include <functional>
#include <memory>

#include "../utils/Time.h"

namespace HybridAStar
{
  namespace Common
  {
    /** \brief Signature for functions that decide whether termination
        conditions have been met for a planner, even if no
        solution is found. This is usually reaching a time or
        memory limit. If the function returns true, the planner is
        signaled to terminate its computation. Otherwise,
        computation continues while this function returns false,
        until a solution is found. */
    using PlannerTerminationConditionFn = std::function<bool()>;
    /** \brief Encapsulate a termination condition for a motion
        planner. Planners will call operator() to decide whether
        they should terminate before a solution is found or
        not. operator() will return true if either the implemented
        condition is met (the call to eval() returns true) or if
        the user called terminate(true). */
    class PlannerTerminationCondition
    {
     public:
        /** \brief Construct a termination condition. By default, eval() will call the externally specified function
           \e fn to decide whether
            the planner should terminate. */
        PlannerTerminationCondition(const PlannerTerminationConditionFn &fn);

        /** \brief Construct a termination condition that is evaluated every \e period seconds. The evaluation of
            the condition consists of calling \e fn() in a separate thread. Calls to eval() will always return the
            last value computed by the call to \e fn(). */
        PlannerTerminationCondition(const PlannerTerminationConditionFn &fn, double period);

        ~PlannerTerminationCondition() = default;

        /** \brief Return true if the planner should stop its computation */
        bool operator()() const
        {
            return eval();
        }

        /** \brief Cast as true if the planner should stop its computation */
        operator bool() const
        {
            return eval();
        }

        /** \brief Notify that the condition for termination should become true, regardless of what eval() returns.
            This function may be called while the condition is being evaluated by other threads. */
        void terminate() const;

        /** \brief The implementation of some termination condition. By default, this just calls \e fn_() */
        bool eval() const;

     private:
        class PlannerTerminationConditionImpl;
        std::shared_ptr<PlannerTerminationConditionImpl> impl_;
    };

    /** \brief Simple termination condition that always returns false. The termination condition will never be met
     */
    PlannerTerminationCondition plannerNonTerminatingCondition();

    /** \brief Simple termination condition that always returns true. The termination condition will always be met
     */
    PlannerTerminationCondition plannerAlwaysTerminatingCondition();

    /** \brief Combine two termination conditions into one. If either termination condition returns true, this one
     * will return true as well. */
    PlannerTerminationCondition plannerOrTerminationCondition(const PlannerTerminationCondition &c1,
                                                              const PlannerTerminationCondition &c2);

    /** \brief Combine two termination conditions into one. Both termination conditions need to return true for this
     * one to return true. */
    PlannerTerminationCondition plannerAndTerminationCondition(const PlannerTerminationCondition &c1,
                                                                const PlannerTerminationCondition &c2);

    /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time) */
    PlannerTerminationCondition timedPlannerTerminationCondition(double duration);

    /** \brief Return a termination condition that will become true \e duration in the future (wall-time) */
    PlannerTerminationCondition timedPlannerTerminationCondition(Time::duration duration);

    /** \brief Return a termination condition that will become true \e duration seconds in the future (wall-time),
     * but is checked in a separate thread, every \e interval seconds; \e interval must be less than \e duration */
    PlannerTerminationCondition timedPlannerTerminationCondition(double duration, double interval);

    /** \brief Return a termination condition that will become true as soon as the problem definition has an exact
     * solution */
    // PlannerTerminationCondition exactSolnPlannerTerminationCondition(const ompl::base::ProblemDefinitionPtr &pdef);
  } // namespace Common
} // namespace HybridAStar

#endif