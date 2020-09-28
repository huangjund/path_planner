#include <thread>
#include <utility>
#include <ctime>

#include "PlannerTerminationCondition.h"

namespace HybridAStar
{
namespace Common
{
  class PlannerTerminationCondition::PlannerTerminationConditionImpl
  {
   public:
      PlannerTerminationConditionImpl(PlannerTerminationConditionFn fn, double period)
        : fn_(std::move(fn))
        , period_(period)
        , terminate_(false)
        , thread_(nullptr)
        , evalValue_(false)
        , signalThreadStop_(false)
      {
          if (period_ > 0.0)
              startEvalThread();
      }

      ~PlannerTerminationConditionImpl()
      {
          stopEvalThread();
      }

      bool eval() const
      {
          if (terminate_)
              return true;
          if (period_ > 0.0)
              return evalValue_;
          return fn_();
      }

      void terminate() const
      {
        // it is ok to have unprotected write here
        terminate_ = true;
      }

   private:
      /** \brief Start the thread evaluating termination conditions if not already started */
      void startEvalThread()
      {
        if (thread_ == nullptr)
        {
          signalThreadStop_ = false;
          evalValue_ = false;
          thread_ = new std::thread([this]
                                    {
                                        periodicEval();
                                    }); // start a periodic evaluation thread
        }
      }

      /** \brief Stop the thread evaluating termination conditions if not already stopped */
      void stopEvalThread()
      {
        signalThreadStop_ = true;
        if (thread_ != nullptr)
        {
            thread_->join();  // pause until thread_ finishes
            delete thread_;
            thread_ = nullptr;
        }
      }

      /** \brief Worker function that runs in a separate thread (calls computeEval())*/
      void periodicEval()
      {
        // we want to check for termination at least once every ms;
        // even though we may evaluate the condition itself more rarely

        unsigned int count = 1;
        Time::duration s = Time::seconds(period_);
        if (period_ > 0.001)
        {
            count = 0.5 + period_ / 0.001;
            s = Time::seconds(period_ / (double)count);
        }

        while (!terminate_ && !signalThreadStop_)
        {
          evalValue_ = fn_();
          for (unsigned int i = 0; i < count; ++i)
          {
            if (terminate_ || signalThreadStop_)
                break;
            std::this_thread::sleep_for(s);
          }
        }
      }

      /** \brief Function pointer to the piece of code that decides whether a termination condition has been met
       */
      PlannerTerminationConditionFn fn_;

      /** \brief Interval of time (seconds) to wait between calls to computeEval() */
      double period_;

      /** \brief Flag indicating whether the user has externally requested that the condition for termination
       * should become true */
      mutable bool terminate_;

      /** \brief Thread for periodicEval() */
      std::thread *thread_;

      /** \brief Cached value returned by fn_() ,true if the planner is time to be terminated*/
      bool evalValue_;

      /** \brief Flag used to signal the condition evaluation thread to stop. */
      bool signalThreadStop_;
  };

} // namespace Common
} // namespace HybridAStar

HybridAStar::Common::PlannerTerminationCondition::PlannerTerminationCondition(const PlannerTerminationConditionFn &fn)
: impl_(std::make_shared<PlannerTerminationConditionImpl>(fn, -1.0))
{
}

HybridAStar::Common::PlannerTerminationCondition::PlannerTerminationCondition(const PlannerTerminationConditionFn &fn,
                                                                    double period)
: impl_(std::make_shared<PlannerTerminationConditionImpl>(fn, period))
{
}

void HybridAStar::Common::PlannerTerminationCondition::terminate() const
{
  impl_->terminate();
}

bool HybridAStar::Common::PlannerTerminationCondition::eval() const
{
  return impl_->eval();
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::plannerNonTerminatingCondition()
{
  return PlannerTerminationCondition([]
                                      {
                                          return false;
                                      });
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::plannerAlwaysTerminatingCondition()
{
  return PlannerTerminationCondition([]
                                      {
                                          return true;
                                      });
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::plannerOrTerminationCondition(const PlannerTerminationCondition &c1,
                                                                                const PlannerTerminationCondition &c2)
{
  return PlannerTerminationCondition([c1, c2]
                                      {
                                          return c1() || c2();
                                      });
}

HybridAStar::Common::PlannerTerminationCondition
HybridAStar::Common::plannerAndTerminationCondition(const PlannerTerminationCondition &c1, const PlannerTerminationCondition &c2)
{
  return PlannerTerminationCondition([c1, c2]
                                      {
                                          return c1() && c2();
                                      });
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::timedPlannerTerminationCondition(double duration)
{
  return timedPlannerTerminationCondition(Time::seconds(duration));
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::timedPlannerTerminationCondition(Time::duration duration)
{
  const Time::point endTime(Time::now() + duration);
  return PlannerTerminationCondition([endTime]
                                      {
                                          return Time::now() > endTime;
                                      });
}

HybridAStar::Common::PlannerTerminationCondition HybridAStar::Common::timedPlannerTerminationCondition(double duration, double interval)
{
  if (interval > duration)
      interval = duration;
  const Time::point endTime(Time::now() + Time::seconds(duration));
  return PlannerTerminationCondition([endTime]
                                      {
                                          return Time::now() > endTime;
                                      },
                                      interval);
}

// HybridAStar::Common::PlannerTerminationCondition
// HybridAStar::Common::exactSolnPlannerTerminationCondition(const HybridAStar::Common::ProblemDefinitionPtr& pdef)
// {
//   return PlannerTerminationCondition([pdef]
//                                       {
//                                           return pdef->hasExactSolution();
//                                       });
// }
