#include "RRTXdynamic.h"


namespace HybridAStar
{
namespace Geometry
{
  RRTXdynamic::RRTXdynamic(const point_t& goal, const point_t& start,
                          double width, double height,  //[meters]
                          std::shared_ptr<Common::CollisionDetection>& config):
                          v_goal_(goal),v_start_(start),v_bot_(start),
                          vertexSet_(pointVec{std::make_shared<point_t>(v_goal_)}),
                          visableTree_(kdtree(vertexSet_)),
                          u(std::uniform_real_distribution<double>(0,1)),
                          map_width_(width), map_height_(height) {
    gama = 6*width*height; // 2^D(1+1/D)*S(x_free)
    setStateValidityChecker(config);
    setMotionValidityChecker(config);
  }

  RRTXdynamic::Visualizer::Visualizer() {
    pub = n.advertise<visualization_msgs::Marker>("/dynamicPath", 1);
  }

  void RRTXdynamic::solve(const Common::PlannerTerminationCondition& ptc) {
    double radius = shrinkingBallRadius(vertexSet_.size());
    // termination condition directed tree generation
    while(!ptc) {
      radius = shrinkingBallRadius(vertexSet_.size());
      radius = 1.5;
      updateObstacles();
      auto v = genRandom();
      v[0] *= map_width_; v[1] *= map_height_;
      auto v_nearest = nearest(v);

      // if the new point is greater than the largest radius
      // give the new vertex an offset
      if (Distance(v, *v_nearest) > delta) 
        saturate(v, *v_nearest);

      // extend the new vertex
      std::shared_ptr<point_t> extended_v(nullptr);
      extend(v,radius, extended_v);
      
      if(extended_v) {
        rewireNeighbors(extended_v,radius);
        reduceInconsistency(radius);
      }
    }

    // extend to solution (connect the start to a feasible tree node)
    auto start = std::make_shared<point_t>(v_start_);
    connectNearestFeasible(start, radius);

    { // trace the path
      ptc.terminate();
      std::vector<point_t> path;
      auto temp = start;
      point_t point;
      while(temp->prtTreePositive != NULL) {
        point[0] = (*temp)[0]; point[1] = (*temp)[1];
        path.push_back(point);
        temp = temp->prtTreePositive;
      }
      point[0] = (*temp)[0]; point[1] = (*temp)[1];
      path.push_back(point);

      { // visualization
        visualization_msgs::Marker marker;
        geometry_msgs::Point p_start, p_end;
        marker.header.frame_id = "path";
        marker.header.stamp = ros::Time::now();
        marker.ns = "tree";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = .01;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        visual(visableTree_.rootNode(), marker);
        visualizer.pub.publish(marker);
      }
    }
  }

  void RRTXdynamic::solve(double solvetime) {
    if (solvetime < 1.0)
      return solve(Common::timedPlannerTerminationCondition(solvetime));
    return solve(Common::timedPlannerTerminationCondition(solvetime, std::min(solvetime / 100.0, 0.1)));
  }

  void RRTXdynamic::visual(std::shared_ptr<point_t> currPoint, visualization_msgs::Marker& marker) {
    if (currPoint == NULL) return;

    for (auto child : currPoint->cldTreeNeg) {
      visual(child, marker);
      geometry_msgs::Point p_start, p_end;
      p_start.x = (*currPoint)[0]; p_start.y = (*currPoint)[1];
      p_end.x = (*child)[0]; p_end.y = (*child)[1];
      marker.points.push_back(p_start);
      marker.points.push_back(p_end);
    }
  }

  // connect start point to the nearest feasible point
  void RRTXdynamic::connectNearestFeasible(std::shared_ptr<point_t>& v, const double r) {
    // try to find a feasible point to connect to
    auto v_near = visableTree_.kNNValue(v_start_, 
                  [this](const Common::SE2State* s1, const Common::SE2State* s2) {
                      return motionChecker->checkMotion(s1,s2);
                  });
    if (v_near != NULL) {
      vertexSet_.push_back(v);
      visableTree_.insert(v);
      v->prtTreePositive = v_near;
      v->prtTreePositive->cldTreeNeg.push_back(v);

      // only recoginize v_near as neighbor
      v->nOrgPositive.push_back(v_near);
      v->nOrgNeg.push_back(v_near);
      v_near->nRunPositive.push_back(v);
      v_near->nRunNeg.push_back(v);
    }
    else 
      std::cout << "[ERROR]: cannot find a feasible point" << std::endl;
  }

  double RRTXdynamic::shrinkingBallRadius(const size_t& vsetSize) {
    double temp = sqrt(gama*std::log10(vsetSize+1)/(vsetSize*M_PI));
    return std::min(temp, delta);
  }

  void RRTXdynamic::updateObstacles() {}

  RRTXdynamic::point_t RRTXdynamic::genRandom() {
    Point<2> temp;
    temp[0] = u(randEngine);
    temp[1] = u(randEngine);

    return point_t(temp);
  }

  std::shared_ptr<RRTXdynamic::point_t> RRTXdynamic::nearest(const point_t& v) {
    auto queue = visableTree_.kNNValue(v,(std::size_t)1);
    return queue.dequeueMin();
  }

  void RRTXdynamic::saturate(point_t& _v, const point_t& _v_nearest) {
    double dx = _v[0] - _v_nearest[0];
    double dy = _v[1] - _v_nearest[1];
    double len = sqrt(dx*dx + dy*dy);
    _v[0] = _v_nearest[0] + dx*delta/len;
    _v[1] = _v_nearest[1] + dy*delta/len;
  }

  void RRTXdynamic::setStateValidityChecker(const std::shared_ptr<Common::CollisionDetection>& config) {
    stateChecker.reset(std::make_unique<ValidityChecker>(
                      [config](const Common::SE2State* state)->bool{
                        return config->isTraversable(state,true);
                      }
                      ).release());
  }

  void RRTXdynamic::setMotionValidityChecker(const std::shared_ptr<Common::CollisionDetection>& config) {
    motionChecker.reset(std::make_unique<MotionChecker>(
                          [config](const Common::SE2State* s1, const Common::SE2State* s2)->bool{
                            double v1[2] = {s1->getX(),s1->getY()};
                            double v2[2] = {s2->getX(),s2->getY()};

                            return config->fastSearch(v1,v2);
                          }
                        ).release());
  }

  void RRTXdynamic::extend(point_t& _v, const double r, std::shared_ptr<point_t>& v) {
    Common::SE2State v_new(_v[0],_v[1],0);
    //  if the state is not obstacle district
    if(stateChecker->isValid(&v_new)) {
      auto V_near = visableTree_.kNNValue(_v,r);
      findParent(_v,V_near,r);

      if (!_v.prtTreePositive) return;

      v = std::make_shared<point_t>(_v);
      vertexSet_.push_back(v);
      visableTree_.insert(v);
      v->prtTreePositive->cldTreeNeg.push_back(v);

      // update neighbors of points
      for (size_t i = 0; i < V_near.size(); ++i) {
        auto u = V_near[i];
        auto v0 = std::make_unique<Common::SE2State>((*v)[0],(*v)[1],0);
        auto u0 = std::make_unique<Common::SE2State>((*u)[0],(*u)[1],0);
        if (motionChecker->checkMotion(v0.get(), u0.get())) {
          v->nOrgPositive.push_back(u);
          v->nOrgNeg.push_back(u);
          u->nRunPositive.push_back(v);
          u->nRunNeg.push_back(v);
        }
      }
      return;
    }
    else 
      return;
  }

  void RRTXdynamic::findParent(point_t& _v,const BoundedPQueue<std::shared_ptr<Point<2>>>& U,const double& r) {
    for (size_t i = 0; i < U.size(); ++i)
    {
      // get every element in U
      const auto u = U[i];

      // trajectory between two point is line in default
      auto d_uv = Distance(_v,*u);
      auto v_lmc = _v.getMutableLMC();
      auto u_lmc = u->getLMC();
      auto v0 = std::make_unique<Common::SE2State>(_v[0],_v[1],0);
      auto u0 = std::make_unique<Common::SE2State>((*u)[0], (*u)[1], 0);
      
      if(d_uv<=r && v_lmc > d_uv + u_lmc &&
        motionChecker->checkMotion(v0.get(), u0.get())) {
        _v.prtTreePositive = u;
        v_lmc = d_uv + u_lmc;
      }
    }
  }

  void RRTXdynamic::rewireNeighbors(std::shared_ptr<point_t>& _v, const double& r) {
    if (_v->getG() - _v->getLMC() > epsilon) {
      cullNeighbors(_v, r);

      // two sets : No- and Nr-  ===>  N-
      for (auto u = _v->nOrgNeg.begin(); u != _v->nOrgNeg.end(); ++u) {
        if (*u == _v->prtTreePositive) continue;
        
        auto lmc_u = (*u)->getMutableLMC();
        auto lmc_v = _v->getLMC();
        auto d_uv = Distance(**u, *_v);
        if (lmc_u > lmc_v + d_uv) {
          lmc_u = d_uv + lmc_v;
          makeParentOf(*u,_v);
          if ((*u)->getG() - lmc_u > epsilon)
            verrifyQueue(*u);
        }
      }
      for (auto u = _v->nRunNeg.begin(); u != _v->nRunNeg.end(); ++u) {
        if (*u == _v->prtTreePositive) continue;
        
        auto lmc_u = (*u)->getMutableLMC();
        auto lmc_v = _v->getLMC();
        auto d_uv = Distance(**u, *_v);
        if (lmc_u > lmc_v + d_uv) {
          lmc_u = d_uv + lmc_v;
          makeParentOf(*u,_v);
          if ((*u)->getG() - lmc_u > epsilon)
            verrifyQueue(*u);
        }
      }
    }
  }

  void RRTXdynamic::cullNeighbors(std::shared_ptr<point_t>& _v, const double& r) {
    for(auto u = _v->nRunPositive.begin(); u != _v->nRunPositive.end();) {
      if (_v->prtTreePositive != (*u) && r < Distance(*_v,**u)) {
        // inefficient
        // erase 2 points from each sets
        for (auto i = (*u)->nOrgNeg.begin(); i != (*u)->nOrgNeg.end(); ++i) {
          if (*i == _v) {
            (*u)->nOrgNeg.erase(i);
            break;
          }
        }
        _v->nRunPositive.erase(u);
      }
      else {
        ++u;
      }
    }
  }

  void RRTXdynamic::makeParentOf (std::shared_ptr<point_t>& _v,
                                  std::shared_ptr<point_t>& _u) {
    _u->prtTreePositive = _v;
  }

  void RRTXdynamic::verrifyQueue(std::shared_ptr<point_t>& _u) {
    if (Q_.find(_u) == Q_.end()) 
      Q_.emplace(_u,1);
  }

  void RRTXdynamic::reduceInconsistency(const double& radius) {
    while (Q_.size() > 0) {
      auto v = Q_.begin()->first; // copy the first element to v
      Q_.erase(Q_.begin()); // remove the first element from Q_

      if (v->getG() > v->getLMC() + epsilon) {
        updateLMC(v,radius);
        rewireNeighbors(v,radius);
      }

      v->getMutableG() = v->getLMC();
    }
  }

  void RRTXdynamic::updateLMC(std::shared_ptr<point_t>& _v, const double& r) {
    cullNeighbors(_v, r);

    auto p = _v->prtTreePositive;
    // search through positive neighbors of _v
    // Nr+ + No+  ==>> N+
    for (auto u = _v->nOrgPositive.begin(); u != _v->nOrgPositive.end(); ++u) {
      if (orphanSet_.find(*u) == orphanSet_.end() && 
          (*u)->prtTreePositive != _v && 
          _v->getLMC() > Distance(*_v,**u) + (*u)->getLMC()) {
        p = *u;
      }
    }
    for (auto u = _v->nRunPositive.begin(); u != _v->nRunPositive.end(); ++u) {
      if (orphanSet_.find(*u) == orphanSet_.end() && 
          (*u)->prtTreePositive != _v && 
          _v->getLMC() > Distance(*_v,**u) + (*u)->getLMC()) {
        p = *u;
      }
    }

    makeParentOf(p,_v);
  }

} // namespace Geometry
} // namespace HybridAStar
