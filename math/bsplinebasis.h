#ifndef _HYBRID_A_STAR_B_SPLINE_BASIS_H
#define _HYBRID_A_STAR_B_SPLINE_BASIS_H

#include <vector>
#include <utility>
#include "vector2d.h"

namespace HybridAStar
{
  /**
   * @brief base class for b spline computation
   * 
   * \ref reference to: 计算机辅助几何设计与非均匀有理B样条
   * 
   */
  class bsplinebasis
  {
  private:
    int order;
    std::vector<Vector2D> trajPoints; ///< trajectory points

  public:
    bsplinebasis(const std::vector<Vector2D>& trajp, const int odr = 3);
    ~bsplinebasis() = default;

    /**
     * @brief compute the point value at \p glb_value
     * 
     * @param glb_value global t value
     * @return Vector2D 
     */
    Vector2D compute(double glb_value);

    /**
     * @brief the point value at \p loc on the ith piece
     * 
     * @param loc local t value. t belongs to [0,1)
     * @param piece the i th piece
     * @return Vector2D 
     */
    Vector2D pieceiValue(const double loc,const unsigned int piece);

    /**
     * @brief Set the Order of this piece
     * 
     * @param order 
     */
    void setOrder(const int& order);
    size_t trajSize() const {return trajPoints.size();}
  private:
    // order 2
    double n2i2[3][3] ={1 , 0 , 0,
                        -2, 2 , 0,
                        1 , -2, 1};

    double n3i2[3][3] ={1 , 0   , 0,
                        -2, 2   , 0,
                        1 , -1.5, 0.5};

    double n3in[3][3] ={0.5, 0.5 , 0,
                        -1 , 1   , 0,
                        0.5, -1.5, 1};

    double n_i_o2[3][3] ={.5, .5, 0,
                          -1, 1 , 0,
                          .5, -1, .5};

    // order 3
    double n3i3[4][4] ={1 , 0 , 0 , 0,
                        -3, 3 , 0 , 0,
                        3 , -6, 3 , 0,
                        -1, 3 , -3, 1};

    double n4i3[4][4] ={1 , 0    , 0  , 0,
                        -3, 3    , 0  , 0,
                        3 , -4.5 , 1.5, 0,
                        -1, 7.0/4, -1 , .25};

    double n4i4[4][4] ={.25  , .5  , .25   , 0,
                        -0.75, 0   , .75   , 0,
                        .75  , -1.5, .75   , 0,
                        -0.25, 1   , -7.0/4, 1};

    double n5i3[4][4] ={1 , 0    , 0       , 0,
                        -3, 3    , 0       , 0,
                        3 , -4.5 , 1.5     , 0,
                        -1, 7.0/4, -11.0/12, 1.0/6};

    double n5i4[4][4] ={.25  , 7.0/12, 1.0/6  , 0,
                        -0.75, 0.25  , 0.5    , 0,
                        0.75 , -1.25 , 0.5    , 0,
                        -0.25   , 7.0/12, -7.0/12, .25};

    double n5i5[4][4] ={1.0/6 , 7.0/12 , 0.25  , 0,
                        -0.5  , -0.25  , 0.75  , 0,
                        0.5   , -1.25  , 0.75  , 0,
                        -1.0/6, 11.0/12, -7.0/4, 1};

    double n6i3[4][4] ={1 , 0    , 0       , 0,
                        -3, 3    , 0       , 0,
                        3 , -4.5 , 1.5     , 0,
                        -1, 7.0/4, -11.0/12, 1.0/6};

    double n6i4[4][4] ={.25  , 7.0/12 , 1.0/6   , 0,
                        -0.75, .25    , .5      , 0,
                        .75  , -1.25  , .5      , 0,
                        -0.25, 7.0/12 , -0.5    , 1.0/6};

    double n6in_1[4][4] ={1.0/6, 2.0/3, 1.0/6  , 0,
                          -0.5 , 0    , 0.5    , 0,
                          0.5  , -1   , 0.5    , 0,
                          -1.0/6, .5   , -7.0/12, 0.25};

    double n6in[4][4] ={1.0/6 , 7.0/12 , 0.25  , 0,
                        -0.5  , -0.25  , 0.75  , 0,
                        0.5   , -1.25  , 0.75  , 0,
                        -1.0/6, 11.0/12, -7.0/4, 1};

    double n_i_o3[4][4] ={1.0/6 , 2.0/3, 1.0/6, 0,
                          -0.5  , 0    , .5   , 0,
                          .5    , -1   , .5   , 0,
                          -1.0/6, .5   , -0.5 , 1.0/6};
  };  // class bsplinebasis
} // namespace HybridAStar


#endif