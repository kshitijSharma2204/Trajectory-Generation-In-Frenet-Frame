#pragma once

#include "frenet_path.hpp"
#include "polynomial.hpp"   

#include <vector>

namespace trajectory_frenet {

struct LateralState {
  double offset;       
  double speed;        
  double acceleration; 
};

struct LongitudinalState {
  double distance;     
  double speed;       
};

/// Generates & selects optimal trajectories in the Frenet frame
class TrajectoryPlanner {
public:
  TrajectoryPlanner();

  ~TrajectoryPlanner();

  FrenetPath plan(const LateralState& lateral_state,
                  const LongitudinalState& longitudinal_state,
                  std::vector<std::vector<double>>& center_lane,
                  std::vector<std::vector<double>>& obstacles,
                  std::vector<FrenetPath>& all_paths);

  void generateCandidatePaths(const LateralState& lateral_state,
                              const LongitudinalState& longitudinal_state,
                              std::vector<FrenetPath>& candidate_paths);

  void convertToWorldCoordinates(std::vector<FrenetPath>& candidate_paths, 
                                 const std::vector<std::vector<double>>& center_lane);

  void computeCost(FrenetPath& path);

  std::vector<FrenetPath> getValidPaths(const std::vector<FrenetPath>& candidate_paths, 
                                        const std::vector<std::vector<double>>& obstacles);

  bool checkCollision(const FrenetPath& path, const std::vector<std::vector<double>>& obstacles);

  bool checkKinematicConstraints(const FrenetPath& path) const;
};

} // namespace trajectory_frenet
