#include "clsDetection.h"


namespace HybridAStar{
namespace Common{
  CollisionDetection::CollisionDetection():
    carPlant_(std::make_unique<Multibody::SingleForkLiftPlant>()),
    grid_(nullptr),
    collisionLookup_(new configuration[carPlant_->headings_*carPlant_->positions_]()){}

  CollisionDetection::CollisionDetection(nav_msgs::OccupancyGrid::Ptr &grid):
    carPlant_(std::make_unique<Multibody::SingleForkLiftPlant>()),
    grid_(grid),collisionLookup_(new configuration[carPlant_->headings_*carPlant_->positions_]()) {
    makeClsLookup(); // initialize collision lookup
  }

  CollisionDetection::CollisionDetection(const CollisionDetection& cp):
    carPlant_(std::make_unique<Multibody::SingleForkLiftPlant>()),
    grid_(cp.grid_),collisionLookup_(new configuration[carPlant_->headings_*carPlant_->positions_]()) {
      // copy collision lookup
      for (size_t i = 0; i < carPlant_->headings_*carPlant_->positions_; ++i) {
        collisionLookup_[i].length = cp.collisionLookup_[i].length;
        for (size_t j = 0; j < collisionLookup_[i].length; j++)
        {
          collisionLookup_[i].pos[j].x = cp.collisionLookup_[i].pos[j].x;
          collisionLookup_[i].pos[j].y = cp.collisionLookup_[i].pos[j].y;
        }
      }
  }

	CollisionDetection::~CollisionDetection() {
		delete [] collisionLookup_;
  }

  void CollisionDetection::setGrid(nav_msgs::OccupancyGrid::Ptr &grid) {
    grid_ = grid;

    setPGrid();
  }

  void CollisionDetection::setPGrid() {
    auto ccsize = grid_->info.resolution; // collision cell size
    auto pcsize = carPlant_->planResolution;
    pGrid_.height = (int)(grid_->info.height*ccsize/pcsize);
    pGrid_.width = (int)(grid_->info.width*ccsize/pcsize);

    // initailize the grid pointer
    pGrid_.planGrid.reset(new float[pGrid_.height*pGrid_.width]);
    auto pg = pGrid_.planGrid;
    for(int i = 0; i < pGrid_.width; i++){
      for(int j = 0; j < pGrid_.height; j++){
        *(pg.get()+j*pGrid_.width+i) = 0;
      }
    }

    int cpx, cpy; // the current planning map point along x or y direction
    // computing occupancy rate
    for (int i = 0; i < grid_->info.width; i++){
      for (int j = 0; j < grid_->info.height; j++){
        if(grid_->data[j*grid_->info.width + i]){ // if the cell has obstacle, add 
          cpx = static_cast<int>(i*ccsize/pcsize);
          cpy = static_cast<int>(j*ccsize/pcsize);
          *(pg.get()+cpy*pGrid_.width + cpx) += ccsize*ccsize;
        }    
      }
    }
    
    // attain the percent
    for(int i = 0; i < pGrid_.width; i++){
      for(int j = 0; j < pGrid_.height; j++){
        *(pg.get()+j*pGrid_.width + i) /= pcsize*pcsize;
      }
    }
  }

  void CollisionDetection::getConfiguration(const SE2State* state, float &x,float &y, float &t) {
    x = state->getX();
    y = state->getY();
    t = state->getT();
  }


  void CollisionDetection::getConfiguration(const GridState* state, float &x,float &y,float &) {
    x = state->getX();
    y = state->getY();
  }
  
  template <class T>
  bool CollisionDetection::isTraversable(const T* state) {
    float x, y, t;
    getConfiguration(state,x,y,t);
    // TODO:change this into an enumeration struct
    switch (state->dimension)
    {
    case 2:
      if (*(pGrid_.planGrid.get()+state->getIdx()) > pGrid_.threshold){
        return false;
      }
      return true;
      break;
    
    case 3:
      return configinCFree(x, y, t);
      break;

    default:
      exit(0);
      break;
    }
  }

  template bool CollisionDetection::isTraversable<SE2State>(const SE2State*);
  template bool CollisionDetection::isTraversable<GridState>(const GridState*);

  template <class T>
  bool CollisionDetection::isTraversable(const T* state, const bool rrtmap) {
    float x,y,t;
    getConfiguration(state,x,y,t);
    int idx = (int)(y/grid_->info.resolution)*grid_->info.width + (int)(x/grid_->info.resolution);
    return !static_cast<bool>(grid_->data[idx]);
  }

  template bool CollisionDetection::isTraversable<SE2State>(const SE2State*,const bool);

// find whether the line between s1 and s2 collide with obstacle
  bool CollisionDetection::fastSearch(double* s1, double* s2) {
    float csize = grid_->info.resolution;
    int width = grid_->info.width;
    int height = grid_->info.height;
    float start[] = {s1[0]/csize, s1[1]/csize}; // relative point [unit: cell]
    float goal[] = {s2[0]/csize, s2[1]/csize};

    float dx = goal[0] - start[0];
    float dy = goal[1] - start[1];
    auto stepX = sign(dx);
    auto stepY = sign(dy);
    float tDeltaX,tDeltaY;
    double tMaxX, tMaxY;
    int X = static_cast<int>(start[0]);
    int Y = static_cast<int>(start[1]);

    // width and height normalized by t
    if (dx != 0) {
      tDeltaX = 1.f / std::abs(dx);
    } else {
      tDeltaX = std::numeric_limits<double>::max();
    }
    if (dy != 0) {
      tDeltaY = 1.f / std::abs(dy);
    } else {
      tDeltaY = std::numeric_limits<double>::max();
    }

    // set maximum traversal values
    if (stepX > 0) {
      tMaxX = tDeltaX * (1 - (start[0] - (long)start[0]));
    } else {
      tMaxX = tDeltaX * (start[0] - (long)start[0]);
    }

    if (stepY > 0) {
      tMaxY = tDeltaY * (1 - (start[1] - (long)start[1]));
    } else {
      tMaxY = tDeltaY * (start[1] - (long)start[1]);
    }

    while ((int)goal[0] != X || (int)goal[1] != Y) {
      // only increment x if the t length is smaller and the result will be closer to the goal
      if (tMaxX < tMaxY && std::abs(X + stepX - (int)goal[0]) < std::abs(X - (int)goal[0])) {
        tMaxX = tMaxX + tDeltaX;
        X = X + stepX;
        // only increment y if the t length is smaller and the result will be closer to the goal
      } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)goal[1]) < std::abs(Y - (int)goal[1])) {
        tMaxY = tMaxY + tDeltaY;
        Y = Y + stepY;
      } else if (2 >= std::abs(X - (int)goal[0]) + std::abs(Y - (int)goal[1])) {
        if (std::abs(X - (int)goal[0]) > std::abs(Y - (int)goal[1])) {
          X = X + stepX;
        } else {
          Y = Y + stepY;
        }
      } else {
        // this SHOULD NOT happen
        std::cout << "\n--->tie occured, please check for error in script\n";
        return false;
      }
      if(grid_->data[Y*width + X]) return false;
    }
    return true;
  }

  bool CollisionDetection::configinCFree(float x, float y, float t) {
    int X = (int)(x/grid_->info.resolution); // [unit:collision cell]
    int Y = (int)(y/grid_->info.resolution); // [unit:collision cell]
    int iX = (int)((x - (long)x) * carPlant_->positionResolution_);
    iX = iX > 0 ? iX : 0;
    int iY = (int)((y - (long)y) * carPlant_->positionResolution_);
    iY = iY > 0 ? iY : 0;
    int iT = (int)(t / carPlant_->deltaHeadingRad_);
    int idx = iY * carPlant_->positionResolution_ * carPlant_->headings_ + iX * carPlant_->headings_ + iT;
    int cX;
    int cY;

    for (int i = 0; i < collisionLookup_[idx].length; ++i) {
      cX = (X + collisionLookup_[idx].pos[i].x);
      cY = (Y + collisionLookup_[idx].pos[i].y);

      // make sure the configuration coordinates are actually on the grid
      if (cX >= 0 && (unsigned int)cX < grid_->info.width && cY >= 0 && (unsigned int)cY < grid_->info.height) {
        if (grid_->data[cY * grid_->info.width + cX]) {
          return false;
        }
      }
    }

    return true;
  }

	inline int CollisionDetection::sign(double x) {
			if (x >= 0) { return 1; }
			else { return -1; }
	}

  void CollisionDetection::makeClsLookup() {
    std::cout << "I am building the collision lookup table...";
    // collision cell size [unit: meters/cell]
    const float cSize = grid_->info.resolution;
    // bounding box size length/width
		/// [unit: collision cells] -- The bounding box size length and width to precompute all possible headings
		const int bbSize = std::ceil((sqrt(carPlant_->width_*carPlant_->width_ + carPlant_->length_*carPlant_->length_)) / cSize);

    struct point {
      double x;
      double y;
    };

    // ______________________
    // VARIABLES FOR ROTATION
    //center of the rectangle
    point c;
    point temp;
    // points of the rectangle
    point p[4];
    point nP[4];

    // turning angle
    double theta;

    // ____________________________
    // VARIABLES FOR GRID TRAVERSAL
    // vector for grid traversal
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;
    // grid
    bool cSpace[bbSize * bbSize];
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;

    // _____________________________
    // VARIABLES FOR LOOKUP CREATION
    int count = 0;
    const int positionResolution = carPlant_->positionResolution_;
    const int positions = carPlant_->positions_;
    point points[positions];

    // generate all discrete positions within one cell
    for (int i = 0; i < positionResolution; ++i) {
      for (int j = 0; j < positionResolution; ++j) {
        points[positionResolution * i + j].x = 1.f / positionResolution * j;
        points[positionResolution * i + j].y = 1.f / positionResolution * i;
      }
    }


    for (int q = 0; q < positions; ++q) {
      // set the starting angle to zero;
      theta = 0;
      // set points of rectangle
      c.x = (double)bbSize / 2 + points[q].x;
      c.y = (double)bbSize / 2 + points[q].y;

      p[0].x = c.x - carPlant_->length_ / 2 / cSize;
      p[0].y = c.y - carPlant_->width_ / 2 / cSize;

      p[1].x = c.x - carPlant_->length_ / 2 / cSize;
      p[1].y = c.y + carPlant_->width_ / 2 / cSize;

      p[2].x = c.x + carPlant_->length_ / 2 / cSize;
      p[2].y = c.y + carPlant_->width_ / 2 / cSize;

      p[3].x = c.x + carPlant_->length_ / 2 / cSize;
      p[3].y = c.y - carPlant_->width_ / 2 / cSize;

      for (int o = 0; o < carPlant_->headings_; ++o) {
        // initialize cSpace
        for (int i = 0; i < bbSize; ++i) {
          for (int j = 0; j < bbSize; ++j) {
            cSpace[i * bbSize + j] = false;
          }
        }

        // shape rotation
        for (int j = 0; j < 4; ++j) {
          // translate point to origin
          temp.x = p[j].x - c.x;
          temp.y = p[j].y - c.y;

          // rotate and shift back
          nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
          nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
        }

        // create the next angle
        theta += carPlant_->deltaHeadingRad_;

        // cell traversal clockwise
        for (int k = 0; k < 4; ++k) {
          // create the vectors clockwise
          if (k < 3) {
            start = nP[k]; // [unit: collision cell]
            end = nP[k + 1];
          } else {
            start = nP[k];
            end = nP[0];
          }

          //set indexes
          X = (int)start.x;
          Y = (int)start.y;
          cSpace[Y * bbSize + X] = true;
          t.x = end.x - start.x;
          t.y = end.y - start.y;
          stepX = sign(t.x);
          stepY = sign(t.y);

          // width and height normalized by t
          if (t.x != 0) {
            tDeltaX = 1.f / std::abs(t.x);
          } else {
            tDeltaX = std::numeric_limits<double>::max();
          }

          if (t.y != 0) {
            tDeltaY = 1.f / std::abs(t.y);
          } else {
            tDeltaY = std::numeric_limits<double>::max();
          }

          // set maximum traversal values
          if (stepX > 0) {
            tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
          } else {
            tMaxX = tDeltaX * (start.x - (long)start.x);
          }

          if (stepY > 0) {
            tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
          } else {
            tMaxY = tDeltaY * (start.y - (long)start.y);
          }

          while ((int)end.x != X || (int)end.y != Y) {
            // only increment x if the t length is smaller and the result will be closer to the goal
            if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
              tMaxX = tMaxX + tDeltaX;
              X = X + stepX;
              cSpace[Y * bbSize + X] = true;
              // only increment y if the t length is smaller and the result will be closer to the goal
            } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
              tMaxY = tMaxY + tDeltaY;
              Y = Y + stepY;
              cSpace[Y * bbSize + X] = true;
            } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
              if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
                X = X + stepX;
                cSpace[Y * bbSize + X] = true;
              } else {
                Y = Y + stepY;
                cSpace[Y * bbSize + X] = true;
              }
            } else {
              // this SHOULD NOT happen
              std::cout << "\n--->tie occured, please check for error in script\n";
              break;
            }
          }
        }

        // // FILL THE SHAPE
        // for (int i = 0; i < bbSize; ++i) {
        //   // set inside to false
        //   inside = false;

        //   for (int j = 0; j < bbSize; ++j) {

        //     // determine horizontal crossings
        //     for (int k = 0; k < bbSize; ++k) {
        //       if (cSpace[i * bbSize + k] && !inside) {
        //         hcross1 = k;
        //         inside = true;
        //       }

        //       if (cSpace[i * bbSize + k] && inside) {
        //         hcross2 = k;
        //       }
        //     }

        //     // if inside fill
        //     if (j > hcross1 && j < hcross2 && inside) {
        //       cSpace[i * bbSize + j] = true;
        //     }
        //   }
        // }

        // GENERATE THE ACTUAL LOOKUP
        count = 0;

        for (int i = 0; i < bbSize; ++i) {
          for (int j = 0; j < bbSize; ++j) {
            if (cSpace[i * bbSize + j]) {
              // compute the relative position of the car cells
              collisionLookup_[q * carPlant_->headings_ + o].pos[count].x = j - (int)c.x;
              collisionLookup_[q * carPlant_->headings_ + o].pos[count].y = i - (int)c.y; // [unit:cell]
              // add one for the length of the current list
              count++;
            }
          }
        }

        collisionLookup_[q * carPlant_->headings_ + o].length = count;

      }
    }

    std::cout << " done!" << std::endl;
  }
} // namespace Common
} // namespace HybridAStar
