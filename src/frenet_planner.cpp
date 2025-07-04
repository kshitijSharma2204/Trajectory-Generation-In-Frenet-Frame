#include "trajectory_frenet/frenet_planner.hpp"
#include "trajectory_frenet/planner_params.hpp"
#include "trajectory_frenet/frenet_path.hpp"
#include "trajectory_frenet/polynomial.hpp"

#include <cmath>


namespace trajectory_frenet {
TrajectoryPlanner::TrajectoryPlanner() {}

TrajectoryPlanner::~TrajectoryPlanner() {}

FrenetPath TrajectoryPlanner::plan(const LateralState& lateral_state,
                                  const LongitudinalState& longitudinal_state,
                                  std::vector<std::vector<double>>& center_lane,
                                  std::vector<std::vector<double>>& obstacles,
                                  std::vector<FrenetPath>& all_paths) {

  std::vector<FrenetPath> candidate_paths;
  generateCandidatePaths(lateral_state, longitudinal_state, candidate_paths);

  convertToWorldCoordinates(candidate_paths, center_lane);

  // check validity of the candidate paths
  std::vector<FrenetPath> feasible_paths;
  feasible_paths = getValidPaths(candidate_paths, obstacles);

  all_paths = std::move(candidate_paths);

  // get the best path out of all feasible paths based on cost
  FrenetPath best_path;
  
  double total_cost = std::numeric_limits<double>::max();

  for (auto& path : feasible_paths) {
    if (total_cost >= path.total_cost) {
      total_cost = path.total_cost;
      best_path = path;
    }
  }
  return best_path;
}

void TrajectoryPlanner::generateCandidatePaths(const LateralState& lateral_state,
                                               const LongitudinalState& longitudinal_state,
                                               std::vector<FrenetPath>& candidate_paths) {
  
  // candidate_paths.clear();
  
  VehicleParams vp;
  SamplingParams sp;
  CostWeights cw;

  // Iterate over prediction horizon range 
  for (double T = sp.min_prediction_limit; T < sp.max_prediction_limit; T += sp.horizon_step) {

    // Iterate over different lanes for a given time horizon
    double max_offset = ((sp.n_lanes-1) * sp.lane_width) / 2.0;
    for (double lat_offset = -max_offset; lat_offset <= max_offset; lat_offset += sp.lane_width) {
      double lat_speed = 0.0, lat_acceleration = 0.0;
      
      std::vector<std::vector<double>> lat_traj;
      Eigen::Matrix<double, 6, 1> lat_boundaries;
      lat_boundaries << 
        lateral_state.offset,             // Initial lateral offset
        lateral_state.speed,              // Initial lateral speed
        lateral_state.acceleration,       // Initial lateral acceleration
        lat_offset,                       // Target lateral offset
        lat_speed,                        // Target lateral speed
        lat_acceleration;                 // Target lateral acceleration
      Polynomial lat_traj_poly(lat_boundaries, T);
      
      // Iterate over time steps for a given lane and a given time horizon 
      // to generate lateral trajectory points 
      double lateral_jerk_cost = 0.0;
      for (double t = 0; t <= T; t += sp.time_step) {
        auto p = lat_traj_poly.evaluate_position(t);
        auto v = lat_traj_poly.evaluate_velocity(t);
        auto a = lat_traj_poly.evaluate_acceleration(t);
        auto j = lat_traj_poly.evaluate_jerk(t);
        lateral_jerk_cost += j * j;
        lat_traj.push_back({p, v, a, j, t});
      }
      
      // Iterate over longitudinal velocity range for a given lane and a given time horizon
      for (double lon_speed = sp.target_velocity - sp.velocity_step;
                  lon_speed <= sp.target_velocity + sp.velocity_step;
                  lon_speed += sp.velocity_step) {
        
        FrenetPath candidate;
        candidate.T = T;
        candidate.d = lat_traj;
        candidate.lateral_jerk_cost = lateral_jerk_cost;

        std::vector<std::vector<double>> lon_traj;
        Eigen::Matrix<double, 5, 1> lon_boundaries;
        
        double lon_acceleration = 0.0;
        lon_boundaries <<
          longitudinal_state.distance,      // Initial longitudinal distance
          longitudinal_state.speed,         // Initial longitudinal speed
          0.0,                              // Initial longitudinal acceleration  
          lon_speed,                        // Target longitudinal speed      
          lon_acceleration;                 // Target longitudinal acceleration
        
        Polynomial lon_traj_poly(lon_boundaries, T);

        // Iterate over time steps for a given longitudinal velocity and a lateral trajectory
        // to generate longitudinal trajectory points
        double longitudinal_jerk_cost = 0.0;
        for (double t = 0; t <= T; t += sp.time_step) {
          auto p = lon_traj_poly.evaluate_position(t);
          auto v = lon_traj_poly.evaluate_velocity(t);
          auto a = lon_traj_poly.evaluate_acceleration(t);
          auto j = lon_traj_poly.evaluate_jerk(t);
          longitudinal_jerk_cost += j * j;
          if (v > candidate.max_velocity) {
            candidate.max_velocity = v;
          }
          if (a > candidate.max_acceleration) {
            candidate.max_acceleration = a;
          }
          lon_traj.push_back({p, v, a, j, t});
        }
        
        candidate.s = lon_traj;
        candidate.longitudinal_jerk_cost = longitudinal_jerk_cost;
        
        computeCost(candidate);
        candidate_paths.push_back(candidate);
      }
    }
  }
}

void TrajectoryPlanner::convertToWorldCoordinates(std::vector<FrenetPath>& candidate_paths,
                                                  const std::vector<std::vector<double>>& center_lane) {

  // calculate x,y in world frame
  for (auto& candidate : candidate_paths) {
    int j = 0;
    for (int i = 0; i < candidate.s.size(); ++i) {
      double x, y, yaw;
      for (; j < center_lane.size(); ++j) {
        if (std::abs(candidate.s[i][0] - center_lane[j][4]) <= 0.1) {
          x = center_lane[j][0] + candidate.d[i][0] * std::cos(center_lane[j][2] + 1.57);
          y = center_lane[j][1] + candidate.d[i][0] * std::sin(center_lane[j][2] + 1.57);
          break;
        }
      }
      candidate.world.push_back({x,y,0,0});
    }
    
    // calculate yaw in world frame
    for (int i = 0; i < candidate.world.size()-1; ++i) {
      candidate.world[i][2] = std::atan2((candidate.world[i+1][1] - candidate.world[i][1]),
                                         (candidate.world[i+1][0] - candidate.world[i][0]));

      candidate.world[i][3] = std::hypot((candidate.world[i+1][0] - candidate.world[i][0]),
                                         (candidate.world[i+1][1] - candidate.world[i][1]));
    }
    candidate.world[candidate.world.size()-1][2] = candidate.world[candidate.world.size()-2][2];
    candidate.world[candidate.world.size()-1][3] = candidate.world[candidate.world.size()-2][3];

    // calculate maximum curvature for the trajectory
    double curvature = std::numeric_limits<double>::min();
    for (int i = 0; i < candidate.world.size()-1; ++i) {
      double temp = std::abs(candidate.world[i+1][2] - candidate.world[i][2]) / (candidate.world[i][3]);
      if (curvature < temp) {
        curvature = temp;
      }
    }
    candidate.max_curvature = curvature;
  }
}

std::vector<FrenetPath> TrajectoryPlanner::getValidPaths(const std::vector<FrenetPath>& candidate_paths, 
                                                         const std::vector<std::vector<double>>& obstacles) {
  std::vector<FrenetPath> feasible_paths;
  for (auto& candidate : candidate_paths) {
    if (!checkCollision(candidate, obstacles) && checkKinematicConstraints(candidate)) {
      feasible_paths.push_back(candidate);
    }
  }
  return feasible_paths;
}

bool TrajectoryPlanner::checkCollision(const FrenetPath& candidate_path, 
                                       const std::vector<std::vector<double>>& obstacles) {
  // Precompute squared collision radius once (e.g. robot footprint + obstacle radius)
  VehicleParams vp;
  const double r2 = vp.robot_footprint * vp.robot_footprint;

  // For each sampled point along the trajectory…
  for (const auto& wp : candidate_path.world) {
      double x = wp[0], y = wp[1];

      // …check against every obstacle
      for (const auto& obs : obstacles) {
          double dx = x - obs[0];
          double dy = y - obs[1];

          // compare squared distances (avoids sqrt())
          if (dx*dx + dy*dy <= r2) {
              return true;
          }
      }
  }

  return false;
}

bool TrajectoryPlanner::checkKinematicConstraints(const FrenetPath& candidate_path) const {
  VehicleParams vp;
  return candidate_path.max_velocity <= vp.max_velocity &&
         candidate_path.max_acceleration <= vp.max_acceleration &&
         candidate_path.max_curvature <= vp.max_curvature; 
}

void TrajectoryPlanner::computeCost(FrenetPath& path) {
  
  CostWeights cw;
  // Lateral cost terms
  const double terminal_lateral_offset = path.d.back()[0];
  const double lateral_jerk_penalty     = cw.kjd * path.lateral_jerk_cost;
  const double lateral_time_penalty     = cw.ktd * path.T;
  const double lateral_deviation_penalty= cw.ksd * (terminal_lateral_offset * terminal_lateral_offset);

  const double lateral_cost =
        lateral_jerk_penalty
      + lateral_time_penalty
      + lateral_deviation_penalty;

  // Longitudinal cost terms
  const double initial_longitudinal_pos = path.s.front()[0];
  const double final_longitudinal_pos   = path.s.back()[0];
  const double longitudinal_jerk_penalty= cw.kjs * path.longitudinal_jerk_cost;
  const double longitudinal_time_penalty= cw.kts * path.T;
  const double longitudinal_error_penalty = cw.kss * ((initial_longitudinal_pos - final_longitudinal_pos) 
                                                   * (initial_longitudinal_pos - final_longitudinal_pos));

  const double longitudinal_cost = longitudinal_jerk_penalty
                                 + longitudinal_time_penalty
                                 + longitudinal_error_penalty;

  // Total cost = weighted sum
  path.total_cost = cw.klat * lateral_cost + cw.klon * longitudinal_cost;
}

} // namespace trajectory_frenet