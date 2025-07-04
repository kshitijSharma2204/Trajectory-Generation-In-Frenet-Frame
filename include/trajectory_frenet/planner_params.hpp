#pragma once

#include <cstddef>

namespace trajectory_frenet {

/// Vehicle physical and dynamic limits
struct VehicleParams {
    double max_velocity = 50.0 / 3.6;  ///< [m/s] Maximum allowed vehicle speed
    double max_acceleration = 2.0;     ///< [m/s²] Maximum allowed vehicle acceleration
    double max_steering_angle = 0.7;   ///< [rad] Maximum steering angle
    double max_curvature = 1.0;        ///< [1/m]  Maximum curvature (1/min_turn_radius)
    double robot_footprint = 3.0;      ///< [m]    Radius for collision checking
};

/// Sampling parameters for trajectory generation
struct SamplingParams {
    double min_prediction_limit = 4.0;    ///< [s] Minimum prediction horizon
    double max_prediction_limit = 5.0;    ///< [s] Maximum prediction horizon
    double n_lanes = 5.0;                 ///< Number of lanes to sample laterally
    double lane_width = 4.0;              ///< [m] Lateral lane offset step size
    double target_velocity = 30.0 / 3.6;  ///< [m/s] Desired longitudinal speed
    double velocity_step = 5.0 / 3.6;     ///< [m/s] Speed sweep around target
    double time_step = 0.1;               ///< [s] Time resolution for trajectory sampling
    double horizon_step = 0.2;            ///< [s] Step size for varying prediction horizon
};

/// Weights for different cost components in path scoring
struct CostWeights {
    double klat = 1.0;   ///< Weight for lateral cost components
    double klon = 1.0;   ///< Weight for longitudinal cost components

    double kjd = 0.1;   ///< Weight for lateral jerk cost
    double ktd = 0.1;   ///< Weight for lateral time cost
    double ksd = 2.0;   ///< Weight for lateral deviation cost

    double kjs = 0.1;   ///< Weight for longitudinal jerk cost
    double kts = 0.1;   ///< Weight for longitudinal time cost
    double kss = 2.0;   ///< Weight for longitudinal deviation cost
};

} // namespace trajectory_frenet