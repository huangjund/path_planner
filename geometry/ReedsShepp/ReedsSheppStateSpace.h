#ifndef OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_
#define OMPL_BASE_SPACES_REEDS_SHEPP_STATE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

namespace HybridAStar
{
    /** \brief An SE(2) state space where distance is measured by the
        length of Reeds-Shepp curves.

        The notation and solutions are taken from:
        J.A. Reeds and L.A. Shepp, “Optimal paths for a car that goes both
        forwards and backwards,” Pacific Journal of Mathematics,
        145(2):367–393, 1990.
        */
    class ReedsSheppStateSpace : public ompl::base::SE2StateSpace
    {
     public:
        /** \brief The Reeds-Shepp path segment types */
        enum ReedsSheppPathSegmentType
        {
            RS_NOP = 0,
            RS_LEFT = 1,
            RS_STRAIGHT = 2,
            RS_RIGHT = 3
        };
        /** \brief Reeds-Shepp path types */
        static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
        /** \brief Complete description of a ReedsShepp path */
        class ReedsSheppPath
        {
          public:
          ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                          double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                          double w = 0., double x = 0.);
          double length() const
          {
              return totalLength_;
          }

          /** Path segment types */
          const ReedsSheppPathSegmentType *type_;
          /** Path segment lengths */
          double length_[5];
          /** Total length */
          double totalLength_;
        };

        ReedsSheppStateSpace(double turningRadius = 1.0) : rho_(turningRadius)
        {
        }

        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;
        virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, bool &firstTime,
                                    ReedsSheppPath &path, ompl::base::State *state) const;

        void sanityChecks() const override
        {
            double zero = std::numeric_limits<double>::epsilon();
            double eps = .1;  // rarely such a large error will occur
            StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
        }

        /** \brief Return the shortest Reeds-Shepp path from SE(2) state state1 to SE(2) state state2 */
        ReedsSheppPath reedsShepp(const ompl::base::State *state1, const ompl::base::State *state2) const;

       protected:
        virtual void interpolate(const ompl::base::State *from, const ReedsSheppPath &path, double t, ompl::base::State *state) const;

        /** \brief Turning radius */
        double rho_;
    };

    /** \brief A Reeds-Shepp motion validator that only uses the state validity checker.
        Motions are checked for validity at a specified resolution.

        This motion validator is almost identical to the DiscreteMotionValidator
        except that it remembers the optimal ReedsSheppPath between different calls to
        interpolate. */
    class ReedsSheppMotionValidator : public ompl::base::MotionValidator
    {
    public:
        ReedsSheppMotionValidator(ompl::base::SpaceInformation *si) : ompl::base::MotionValidator(si)
        {
            defaultSettings();
        }
        ReedsSheppMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si)
        {
            defaultSettings();
        }
        ~ReedsSheppMotionValidator() override = default;
        bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
        bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

    private:
        ReedsSheppStateSpace *stateSpace_;
        void defaultSettings();
    };
}

#endif
