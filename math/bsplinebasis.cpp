#include "bsplinebasis.h"

using namespace HybridAStar;

bsplinebasis::bsplinebasis(const std::vector<Vector2D>& trajp, const int odr):
  order(odr), trajPoints(trajp){}

Vector2D bsplinebasis::compute(double glb_value) {
  int i = (int)glb_value;
  double loc_value = glb_value - (int)glb_value;
  return pieceiValue(loc_value,i);
}

Vector2D bsplinebasis::pieceiValue(const double loc_value, const unsigned int piece) {
  auto n = trajPoints.size() - 1;
  Vector2D temp;
  if (order == 2) {
    auto i = piece + 2;
    double t[3] = {1, loc_value, loc_value*loc_value};

    // switch according to number of trajectory points
    switch (n) {
    case 2: // n == 2; i == 2
      temp = (t[0]*n2i2[0][0]+t[1]*n2i2[1][0]+t[2]*n2i2[2][0])*trajPoints[0] + 
             (t[0]*n2i2[0][1]+t[1]*n2i2[1][1]+t[2]*n2i2[2][1])*trajPoints[1] + 
             (t[0]*n2i2[0][2]+t[1]*n2i2[1][2]+t[2]*n2i2[2][2])*trajPoints[2];
      break;

    // n >= 3
    default:
      if (i == 2) { // n == 3 ; i == 2
        temp = (t[0]*n3i2[0][0]+t[1]*n3i2[1][0]+t[2]*n3i2[2][0])*trajPoints[0] + 
               (t[0]*n3i2[0][1]+t[1]*n3i2[1][1]+t[2]*n3i2[2][1])*trajPoints[1] + 
               (t[0]*n3i2[0][2]+t[1]*n3i2[1][2]+t[2]*n3i2[2][2])*trajPoints[2];
      }
      else if(i == n) { // n >= 3 ; i == n
        temp = (t[0]*n3in[0][0]+t[1]*n3in[1][0]+t[2]*n3in[2][0])*trajPoints[i-2] + 
               (t[0]*n3in[0][1]+t[1]*n3in[1][1]+t[2]*n3in[2][1])*trajPoints[i-1] + 
               (t[0]*n3in[0][2]+t[1]*n3in[1][2]+t[2]*n3in[2][2])*trajPoints[i];
      }
      else { // n > 3 ; 3 <= i <= n-1
        temp = (t[0]*n_i_o2[0][0]+t[1]*n_i_o2[1][0]+t[2]*n_i_o2[2][0])*trajPoints[i-2] + 
               (t[0]*n_i_o2[0][1]+t[1]*n_i_o2[1][1]+t[2]*n_i_o2[2][1])*trajPoints[i-1] + 
               (t[0]*n_i_o2[0][2]+t[1]*n_i_o2[1][2]+t[2]*n_i_o2[2][2])*trajPoints[i];
      }
      break;
    }
  }
  else if(order == 3) {
    auto i = piece + 3;
    double t[4] = {1, loc_value, loc_value*loc_value, loc_value*loc_value*loc_value};

    switch (n) {
    case 3: // n == 3 ; i == 3
      temp = (t[0]*n3i3[0][0]+t[1]*n3i3[1][0]+t[2]*n3i3[2][0]+t[3]*n3i3[3][0])*trajPoints[i-3] + 
             (t[0]*n3i3[0][1]+t[1]*n3i3[1][1]+t[2]*n3i3[2][1]+t[3]*n3i3[3][1])*trajPoints[i-2] + 
             (t[0]*n3i3[0][2]+t[1]*n3i3[1][2]+t[2]*n3i3[2][2]+t[3]*n3i3[3][2])*trajPoints[i-1] + 
             (t[0]*n3i3[0][3]+t[1]*n3i3[1][3]+t[2]*n3i3[2][3]+t[3]*n3i3[3][3])*trajPoints[i];
      break;
    case 4:
      if(i == 3)  // n == 4 ; i == 3;
        temp = (t[0]*n4i3[0][0]+t[1]*n4i3[1][0]+t[2]*n4i3[2][0]+t[3]*n4i3[3][0])*trajPoints[i-3] + 
               (t[0]*n4i3[0][1]+t[1]*n4i3[1][1]+t[2]*n4i3[2][1]+t[3]*n4i3[3][1])*trajPoints[i-2] + 
               (t[0]*n4i3[0][2]+t[1]*n4i3[1][2]+t[2]*n4i3[2][2]+t[3]*n4i3[3][2])*trajPoints[i-1] + 
               (t[0]*n4i3[0][3]+t[1]*n4i3[1][3]+t[2]*n4i3[2][3]+t[3]*n4i3[3][3])*trajPoints[i];
      else if(i == 4) // n == 4 ; i == 4;
        temp = (t[0]*n4i4[0][0]+t[1]*n4i4[1][0]+t[2]*n4i4[2][0]+t[3]*n4i4[3][0])*trajPoints[i-3] + 
               (t[0]*n4i4[0][1]+t[1]*n4i4[1][1]+t[2]*n4i4[2][1]+t[3]*n4i4[3][1])*trajPoints[i-2] + 
               (t[0]*n4i4[0][2]+t[1]*n4i4[1][2]+t[2]*n4i4[2][2]+t[3]*n4i4[3][2])*trajPoints[i-1] + 
               (t[0]*n4i4[0][3]+t[1]*n4i4[1][3]+t[2]*n4i4[2][3]+t[3]*n4i4[3][3])*trajPoints[i];
      else 
        exit(0);
      break;
    case 5:
      if(i == 3)  // n == 5 ; i == 3;
        temp = (t[0]*n5i3[0][0]+t[1]*n5i3[1][0]+t[2]*n5i3[2][0]+t[3]*n5i3[3][0])*trajPoints[i-3] + 
               (t[0]*n5i3[0][1]+t[1]*n5i3[1][1]+t[2]*n5i3[2][1]+t[3]*n5i3[3][1])*trajPoints[i-2] + 
               (t[0]*n5i3[0][2]+t[1]*n5i3[1][2]+t[2]*n5i3[2][2]+t[3]*n5i3[3][2])*trajPoints[i-1] + 
               (t[0]*n5i3[0][3]+t[1]*n5i3[1][3]+t[2]*n5i3[2][3]+t[3]*n5i3[3][3])*trajPoints[i];
      else if(i == 4) // n == 5 ; i == 4
        temp = (t[0]*n5i4[0][0]+t[1]*n5i4[1][0]+t[2]*n5i4[2][0]+t[3]*n5i4[3][0])*trajPoints[i-3] + 
               (t[0]*n5i4[0][1]+t[1]*n5i4[1][1]+t[2]*n5i4[2][1]+t[3]*n5i4[3][1])*trajPoints[i-2] + 
               (t[0]*n5i4[0][2]+t[1]*n5i4[1][2]+t[2]*n5i4[2][2]+t[3]*n5i4[3][2])*trajPoints[i-1] + 
               (t[0]*n5i4[0][3]+t[1]*n5i4[1][3]+t[2]*n5i4[2][3]+t[3]*n5i4[3][3])*trajPoints[i];
      else if(i == 5) // n == 5 ; i == 5
        temp = (t[0]*n5i5[0][0]+t[1]*n5i5[1][0]+t[2]*n5i5[2][0]+t[3]*n5i5[3][0])*trajPoints[i-3] + 
               (t[0]*n5i5[0][1]+t[1]*n5i5[1][1]+t[2]*n5i5[2][1]+t[3]*n5i5[3][1])*trajPoints[i-2] + 
               (t[0]*n5i5[0][2]+t[1]*n5i5[1][2]+t[2]*n5i5[2][2]+t[3]*n5i5[3][2])*trajPoints[i-1] + 
               (t[0]*n5i5[0][3]+t[1]*n5i5[1][3]+t[2]*n5i5[2][3]+t[3]*n5i5[3][3])*trajPoints[i];
      else 
        exit(0);
    default:
      if(i == 3)  // n >= 6 ; i == 3
        temp = (t[0]*n6i3[0][0]+t[1]*n6i3[1][0]+t[2]*n6i3[2][0]+t[3]*n6i3[3][0])*trajPoints[i-3] + 
               (t[0]*n6i3[0][1]+t[1]*n6i3[1][1]+t[2]*n6i3[2][1]+t[3]*n6i3[3][1])*trajPoints[i-2] + 
               (t[0]*n6i3[0][2]+t[1]*n6i3[1][2]+t[2]*n6i3[2][2]+t[3]*n6i3[3][2])*trajPoints[i-1] + 
               (t[0]*n6i3[0][3]+t[1]*n6i3[1][3]+t[2]*n6i3[2][3]+t[3]*n6i3[3][3])*trajPoints[i];
      else if(i == 4) // n >= 6 ; i == 4
        temp = (t[0]*n6i4[0][0]+t[1]*n6i4[1][0]+t[2]*n6i4[2][0]+t[3]*n6i4[3][0])*trajPoints[i-3] + 
               (t[0]*n6i4[0][1]+t[1]*n6i4[1][1]+t[2]*n6i4[2][1]+t[3]*n6i4[3][1])*trajPoints[i-2] + 
               (t[0]*n6i4[0][2]+t[1]*n6i4[1][2]+t[2]*n6i4[2][2]+t[3]*n6i4[3][2])*trajPoints[i-1] + 
               (t[0]*n6i4[0][3]+t[1]*n6i4[1][3]+t[2]*n6i4[2][3]+t[3]*n6i4[3][3])*trajPoints[i];
      else if(i == n-1)  // n >= 6 ; i == n-1
        temp = (t[0]*n6in_1[0][0]+t[1]*n6in_1[1][0]+t[2]*n6in_1[2][0]+t[3]*n6in_1[3][0])*trajPoints[i-3] + 
               (t[0]*n6in_1[0][1]+t[1]*n6in_1[1][1]+t[2]*n6in_1[2][1]+t[3]*n6in_1[3][1])*trajPoints[i-2] + 
               (t[0]*n6in_1[0][2]+t[1]*n6in_1[1][2]+t[2]*n6in_1[2][2]+t[3]*n6in_1[3][2])*trajPoints[i-1] + 
               (t[0]*n6in_1[0][3]+t[1]*n6in_1[1][3]+t[2]*n6in_1[2][3]+t[3]*n6in_1[3][3])*trajPoints[i];
      else if(i == n) // n >= 6 ; i == n;
        temp = (t[0]*n6in[0][0]+t[1]*n6in[1][0]+t[2]*n6in[2][0]+t[3]*n6in[3][0])*trajPoints[i-3] + 
               (t[0]*n6in[0][1]+t[1]*n6in[1][1]+t[2]*n6in[2][1]+t[3]*n6in[3][1])*trajPoints[i-2] + 
               (t[0]*n6in[0][2]+t[1]*n6in[1][2]+t[2]*n6in[2][2]+t[3]*n6in[3][2])*trajPoints[i-1] + 
               (t[0]*n6in[0][3]+t[1]*n6in[1][3]+t[2]*n6in[2][3]+t[3]*n6in[3][3])*trajPoints[i];
      else  // n >= 6 ; 5 <= i <= n-2;
        temp = (t[0]*n_i_o3[0][0]+t[1]*n_i_o3[1][0]+t[2]*n_i_o3[2][0]+t[3]*n_i_o3[3][0])*trajPoints[i-3] + 
               (t[0]*n_i_o3[0][1]+t[1]*n_i_o3[1][1]+t[2]*n_i_o3[2][1]+t[3]*n_i_o3[3][1])*trajPoints[i-2] + 
               (t[0]*n_i_o3[0][2]+t[1]*n_i_o3[1][2]+t[2]*n_i_o3[2][2]+t[3]*n_i_o3[3][2])*trajPoints[i-1] + 
               (t[0]*n_i_o3[0][3]+t[1]*n_i_o3[1][3]+t[2]*n_i_o3[2][3]+t[3]*n_i_o3[3][3])*trajPoints[i];
      break;
    }
  }
  else 
    exit(0);

  return temp;
}


void bsplinebasis::setOrder(const int& i) {
  order = i;
}