#include "RSPath4Fork.h"

using namespace HybridAStar::Geometry;

RSPath4Fork::RSPath4Fork(const double max_kappa, const double step_size) :
                        ReedShepp(max_kappa,step_size) {}

RSPath4Fork::RSPath4Fork(const Common::SE2StatePtr& start_node,
                         const Common::SE2StatePtr& end_node) :
                         ReedShepp(start_node,end_node) {
}

RSPath4Fork::RSPath4Fork(const Common::SE2StatePtr& start_node,
                         const Common::SE2StatePtr& end_node,
                         const double max_kappa,
                         const double step_size):
                         ReedShepp(start_node,end_node,max_kappa,step_size){
}

double RSPath4Fork::ShortestRSPlength() {
  std::vector<ReedSheppPath> all_possible_paths;

  if (!GenerateRSP(&all_possible_paths)) {
    return 0; // if failed to find one feasible path, return length as 0
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  size_t optimal_path_index = 0;

  size_t paths_size = all_possible_paths.size();
  for (size_t i = 0; i < paths_size; ++i) {
    if (all_possible_paths.at(i).total_length > 0 &&
        all_possible_paths.at(i).total_length < 
          optimal_path_length) {  // add threshold 3
      optimal_path_index = i;
      optimal_path_length = all_possible_paths.at(i).total_length / max_kappa_;
    }
  }

  return optimal_path_length;
}

std::vector<ReedSheppPath> RSPath4Fork::possiblePath() {
  std::vector<ReedSheppPath> all_possible_paths;

  if(!GenerateRSP(&all_possible_paths)) {
    all_possible_paths.clear();
    std::cout << "no suitable paths" << std::endl;
  }

  return all_possible_paths;
}

// TODO
bool RSPath4Fork::SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam SLS_param;
  SLS(x, y, phi, &SLS_param);
  double SLS_lengths[3] = {SLS_param.t, SLS_param.u, SLS_param.v};
  char SLS_types[] = "SLS";
  if (SLS_param.flag &&
      !SetRSP(3, SLS_lengths, SLS_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with SLS_param" << std::endl;
    return false;
  }

  RSPParam SRS_param;
  SLS(x, -y, -phi, &SRS_param);
  double SRS_lengths[3] = {SRS_param.t, SRS_param.u, SRS_param.v};
  char SRS_types[] = "SRS";
  if (SRS_param.flag &&
      !SetRSP(3, SRS_lengths, SRS_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with SRS_param" << std::endl;
    return false;
  }
  return true;
}

bool RSPath4Fork::CSC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LSL1_param;
  LSL(x, y, phi, &LSL1_param);
  double LSL1_lengths[3] = {LSL1_param.t, LSL1_param.u, LSL1_param.v};
  char LSL1_types[] = "LSL";
  if (LSL1_param.flag &&
      !SetRSP(3, LSL1_lengths, LSL1_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LSL_param" << std::endl;
    return false;
  }

  RSPParam LSL3_param;
  LSL(x, -y, -phi, &LSL3_param);
  double LSL3_lengths[3] = {LSL3_param.t, LSL3_param.u, LSL3_param.v};
  char LSL3_types[] = "RSR";
  if (LSL3_param.flag &&
      !SetRSP(3, LSL3_lengths, LSL3_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LSL3_param" << std::endl;
    return false;
  }

  RSPParam LSR1_param;
  LSR(x, y, phi, &LSR1_param);
  double LSR1_lengths[3] = {LSR1_param.t, LSR1_param.u, LSR1_param.v};
  char LSR1_types[] = "LSR";
  if (LSR1_param.flag &&
      !SetRSP(3, LSR1_lengths, LSR1_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LSR1_param" << std::endl;
    return false;
  }

  RSPParam LSR3_param;
  LSR(x, -y, -phi, &LSR3_param);
  double LSR3_lengths[3] = {LSR3_param.t, LSR3_param.u, LSR3_param.v};
  char LSR3_types[] = "RSL";
  if (LSR3_param.flag &&
      !SetRSP(3, LSR3_lengths, LSR3_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LSR3_param" << std::endl;
    return false;
  }
  return true;
}

bool RSPath4Fork::CCC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRL1_param;
  LRL(x, y, phi, &LRL1_param);
  double LRL1_lengths[3] = {LRL1_param.t, LRL1_param.u, LRL1_param.v};
  char LRL1_types[] = "LRL";
  if (LRL1_param.flag &&
      !SetRSP(3, LRL1_lengths, LRL1_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL_param" << std::endl;
    return false;
  }

  RSPParam LRL2_param;
  LRL(-x, y, -phi, &LRL2_param);  // timeflip
  double LRL2_lengths[3] = {-LRL2_param.t, -LRL2_param.u, -LRL2_param.v};
  char LRL2_types[] = "LRL";
  if (LRL2_param.flag &&
      !SetRSP(3, LRL2_lengths, LRL2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL2_param" << std::endl;
    return false;
  }

  RSPParam LRL3_param;
  LRL(x, -y, -phi, &LRL3_param);  // reflect
  double LRL3_lengths[3] = {LRL3_param.t, LRL3_param.u, LRL3_param.v};
  char LRL3_types[] = "RLR";
  if (LRL3_param.flag &&
      !SetRSP(3, LRL3_lengths, LRL3_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL3_param" << std::endl;
    return false;
  }

  RSPParam LRL4_param;
  LRL(-x, -y, phi, &LRL4_param);  // timeflip + reflect
  double LRL4_lengths[3] = {-LRL4_param.t, -LRL4_param.u, -LRL4_param.v};
  char LRL4_types[] = "RLR";
  if (LRL4_param.flag &&
      !SetRSP(3, LRL4_lengths, LRL4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL4_param" << std::endl;
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RSPParam LRL5_param;
  LRL(xb, yb, phi, &LRL5_param);
  double LRL5_lengths[3] = {LRL5_param.v, LRL5_param.u, LRL5_param.t};
  char LRL5_types[] = "LRL";
  if (LRL5_param.flag &&
      !SetRSP(3, LRL5_lengths, LRL5_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL5_param" << std::endl;
    return false;
  }

  RSPParam LRL6_param;
  LRL(-xb, yb, -phi, &LRL6_param);
  double LRL6_lengths[3] = {-LRL6_param.v, -LRL6_param.u, -LRL6_param.t};
  char LRL6_types[] = "LRL";
  if (LRL6_param.flag &&
      !SetRSP(3, LRL6_lengths, LRL6_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL6_param" << std::endl;
    return false;
  }

  RSPParam LRL7_param;
  LRL(xb, -yb, -phi, &LRL7_param);
  double LRL7_lengths[3] = {LRL7_param.v, LRL7_param.u, LRL7_param.t};
  char LRL7_types[] = "RLR";
  if (LRL7_param.flag &&
      !SetRSP(3, LRL7_lengths, LRL7_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL7_param" << std::endl;
    return false;
  }

  RSPParam LRL8_param;
  LRL(-xb, -yb, phi, &LRL8_param);
  double LRL8_lengths[3] = {-LRL8_param.v, -LRL8_param.u, -LRL8_param.t};
  char LRL8_types[] = "RLR";
  if (LRL8_param.flag &&
      !SetRSP(3, LRL8_lengths, LRL8_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRL8_param" << std::endl;
    return false;
  }
  return true;
}

bool RSPath4Fork::CCCC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRLRn1_param;
  LRLRn(x, y, phi, &LRLRn1_param);
  double LRLRn1_lengths[4] = {LRLRn1_param.t, LRLRn1_param.u, -LRLRn1_param.u,
                              LRLRn1_param.v};
  char LRLRn1_types[] = "LRLR";
  if (LRLRn1_param.flag &&
      !SetRSP(4, LRLRn1_lengths, LRLRn1_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRn_param" << std::endl;
    return false;
  }

  RSPParam LRLRn2_param;
  LRLRn(-x, y, -phi, &LRLRn2_param);
  double LRLRn2_lengths[4] = {-LRLRn2_param.t, -LRLRn2_param.u, LRLRn2_param.u,
                              -LRLRn2_param.v};
  char LRLRn2_types[] = "LRLR";
  if (LRLRn2_param.flag &&
      !SetRSP(4, LRLRn2_lengths, LRLRn2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRn2_param" << std::endl;
    return false;
  }

  RSPParam LRLRn3_param;
  LRLRn(x, -y, -phi, &LRLRn3_param);
  double LRLRn3_lengths[4] = {LRLRn3_param.t, LRLRn3_param.u, -LRLRn3_param.u,
                              LRLRn3_param.v};
  char LRLRn3_types[] = "RLRL";
  if (LRLRn3_param.flag &&
      !SetRSP(4, LRLRn3_lengths, LRLRn3_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRn3_param" << std::endl;
    return false;
  }

  RSPParam LRLRn4_param;
  LRLRn(-x, -y, phi, &LRLRn4_param);
  double LRLRn4_lengths[4] = {-LRLRn4_param.t, -LRLRn4_param.u, LRLRn4_param.u,
                              -LRLRn4_param.v};
  char LRLRn4_types[] = "RLRL";
  if (LRLRn4_param.flag &&
      !SetRSP(4, LRLRn4_lengths, LRLRn4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRn4_param" << std::endl;
    return false;
  }

  RSPParam LRLRp1_param;
  LRLRp(x, y, phi, &LRLRp1_param);
  double LRLRp1_lengths[4] = {LRLRp1_param.t, LRLRp1_param.u, LRLRp1_param.u,
                              LRLRp1_param.v};
  char LRLRp1_types[] = "LRLR";
  if (LRLRp1_param.flag &&
      !SetRSP(4, LRLRp1_lengths, LRLRp1_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRp1_param" << std::endl;
    return false;
  }

  RSPParam LRLRp2_param;
  LRLRp(-x, y, -phi, &LRLRp2_param);
  double LRLRp2_lengths[4] = {-LRLRp2_param.t, -LRLRp2_param.u, -LRLRp2_param.u,
                              -LRLRp2_param.v};
  char LRLRp2_types[] = "LRLR";
  if (LRLRp2_param.flag &&
      !SetRSP(4, LRLRp2_lengths, LRLRp2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRp2_param" << std::endl;
    return false;
  }

  RSPParam LRLRp3_param;
  LRLRp(x, -y, -phi, &LRLRp3_param);
  double LRLRp3_lengths[4] = {LRLRp3_param.t, LRLRp3_param.u, LRLRp3_param.u,
                              LRLRp3_param.v};
  char LRLRp3_types[] = "RLRL";
  if (LRLRp3_param.flag &&
      !SetRSP(4, LRLRp3_lengths, LRLRp3_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRp3_param" << std::endl;
    return false;
  }

  RSPParam LRLRp4_param;
  LRLRp(-x, -y, phi, &LRLRp4_param);
  double LRLRp4_lengths[4] = {-LRLRp4_param.t, -LRLRp4_param.u, -LRLRp4_param.u,
                              -LRLRp4_param.v};
  char LRLRp4_types[] = "RLRL";
  if (LRLRp4_param.flag &&
      !SetRSP(4, LRLRp4_lengths, LRLRp4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRLRp4_param" << std::endl;
    return false;
  }
  return true;
}

bool RSPath4Fork::CCSC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  // Lt- R+ Su+ Lv+
  RSPParam LRSL2_param;
  LRSL(-x, y, -phi, &LRSL2_param);  // timeflip
  double LRSL2_lengths[4] = {-LRSL2_param.t, 0.5 * M_PI, -LRSL2_param.u,
                             -LRSL2_param.v};
  char LRSL2_types[] = "LRSL";
  if (LRSL2_param.flag &&
      !SetRSP(4, LRSL2_lengths, LRSL2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSL2_param" << std::endl;
    return false;
  }

  // Rt- L+ Su+ Rv+
  RSPParam LRSL4_param;
  LRSL(-x, -y, phi, &LRSL4_param);  // timeflip + reflect
  double LRSL4_lengths[4] = {-LRSL4_param.t, 0.5 * M_PI, -LRSL4_param.u,
                             -LRSL4_param.v};
  char LRSL4_types[] = "RLSR";
  if (LRSL4_param.flag &&
      !SetRSP(4, LRSL4_lengths, LRSL4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSL4_param" << std::endl;
    return false;
  }

  // Lt- R+ Su+ Rv+
  RSPParam LRSR2_param;
  LRSR(-x, y, -phi, &LRSR2_param);  // timeflip
  double LRSR2_lengths[4] = {-LRSR2_param.t, 0.5 * M_PI, -LRSR2_param.u,
                             -LRSR2_param.v};
  char LRSR2_types[] = "LRSR";
  if (LRSR2_param.flag &&
      !SetRSP(4, LRSR2_lengths, LRSR2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSR2_param" << std::endl;
    return false;
  }

  // Rt- L+ Su+ Lv+
  RSPParam LRSR4_param;
  LRSR(-x, -y, phi, &LRSR4_param);  // timeflip + reflect
  double LRSR4_lengths[4] = {-LRSR4_param.t, 0.5 * M_PI, -LRSR4_param.u,
                             -LRSR4_param.v};
  char LRSR4_types[] = "RLSL";
  if (LRSR4_param.flag &&
      !SetRSP(4, LRSR4_lengths, LRSR4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSR4_param" << std::endl;
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  // Lt+ Su+ R+ Lv-
  RSPParam LRSL6_param;
  LRSL(-xb, yb, -phi, &LRSL6_param);  // timeflip
  double LRSL6_lengths[4] = {-LRSL6_param.v, -LRSL6_param.u, 0.5 * M_PI,
                             -LRSL6_param.t};
  char LRSL6_types[] = "LSRL";
  if (LRSL6_param.flag &&
      !SetRSP(4, LRSL6_lengths, LRSL6_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSL6_param" << std::endl;
    return false;
  }

  // Rt+ Su+ L+ Rv-
  RSPParam LRSL8_param;
  LRSL(-xb, -yb, phi, &LRSL8_param);  // timeflip + reflect
  double LRSL8_lengths[4] = {-LRSL8_param.v, -LRSL8_param.u, 0.5 * M_PI,
                             -LRSL8_param.t};
  char LRSL8_types[] = "RSLR";
  if (LRSL8_param.flag &&
      !SetRSP(4, LRSL8_lengths, LRSL8_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSL8_param" << std::endl;
    return false;
  }

  // Rt+ Su+ R+ Lv-
  RSPParam LRSR6_param;
  LRSR(-xb, yb, -phi, &LRSR6_param);  // timeflip
  double LRSR6_lengths[4] = {-LRSR6_param.v, -LRSR6_param.u, 0.5 * M_PI,
                             -LRSR6_param.t};
  char LRSR6_types[] = "RSRL";
  if (LRSR6_param.flag &&
      !SetRSP(4, LRSR6_lengths, LRSR6_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSR6_param" << std::endl;
    return false;
  }

  // Lt+ Su+ L+ Rv-
  RSPParam LRSR8_param;
  LRSR(-xb, -yb, phi, &LRSR8_param);  // timeflip + reflect
  double LRSR8_lengths[4] = {-LRSR8_param.v, -LRSR8_param.u, 0.5 * M_PI,
                             -LRSR8_param.t};
  char LRSR8_types[] = "LSLR";
  if (LRSR8_param.flag &&
      !SetRSP(4, LRSR8_lengths, LRSR8_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSR8_param" << std::endl;
    return false;
  }
  return true;
}

bool RSPath4Fork::CCSCC(const double x, const double y, const double phi,
                      std::vector<ReedSheppPath>* all_possible_paths) {
  // Lt- R+ Su+ L+ Rv-
  RSPParam LRSLR2_param;
  LRSLR(-x, y, -phi, &LRSLR2_param);  // timeflip
  double LRSLR2_lengths[5] = {-LRSLR2_param.t, 0.5 * M_PI, -LRSLR2_param.u,
                              0.5 * M_PI, -LRSLR2_param.v};
  char LRSLR2_types[] = "LRSLR";
  if (LRSLR2_param.flag &&
      !SetRSP(5, LRSLR2_lengths, LRSLR2_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSLR2_param" << std::endl;
    return false;
  }

  // Rt- L+ Su+ R+ Lv-
  RSPParam LRSLR4_param;
  LRSLR(-x, -y, phi, &LRSLR4_param);  // timeflip + reflect
  double LRSLR4_lengths[5] = {-LRSLR4_param.t, 0.5 * M_PI, -LRSLR4_param.u,
                              0.5 * M_PI, -LRSLR4_param.v};
  char LRSLR4_types[] = "RLSRL";
  if (LRSLR4_param.flag &&
      !SetRSP(5, LRSLR4_lengths, LRSLR4_types, all_possible_paths)) {
    std::cout << "Fail at SetRSP with LRSLR4_param" << std::endl;
    return false;
  }
  return true;
}


