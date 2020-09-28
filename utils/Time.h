#ifndef _HYBRID_A_STAR_TIME_H
#define _HYBRID_A_STAR_TIME_H

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace HybridAStar
{
/** \brief Namespace containing time datatypes and time operations */
namespace Time
{
  /** \brief Representation of a point in time */
  using point = std::chrono::system_clock::time_point;

  /** \brief Representation of a time duration */
  using duration = std::chrono::system_clock::duration;

  /** \brief Get the current time point */
  inline point now()
  {
      return std::chrono::system_clock::now();
  }

  /** \brief Return the time duration representing a given number of seconds */
  inline duration seconds(double sec)
  {
      auto s = (long)sec;
      auto us = (long)((sec - (double)s) * 1000000);
      return std::chrono::seconds(s) + std::chrono::microseconds(us);
  }

  /** \brief Return the number of seconds that a time duration represents */
  inline double seconds(const duration &d)
  {
      return std::chrono::duration<double>(d).count();
  }

  /** \brief Return string representation of point in time */
  inline std::string as_string(const point &p)
  {
      std::time_t pt = std::chrono::system_clock::to_time_t(p);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&pt), "%F %T");
      return ss.str();
  }

  // Adapted from the deprecated boost/progress.hpp header file
  class ProgressDisplay
  {
  public:
      explicit ProgressDisplay(std::ostream &os = std::cout);
      unsigned int operator++();
      unsigned int count() const
      {
          return count_;
      }

  private:
      std::ostream& out_;
      unsigned int count_{0u};
  };
} // namespace Time
} // namespace HybridAStar

#endif