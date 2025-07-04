#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>

#include "trajectory_frenet/frenet_planner.hpp"
#include "trajectory_frenet/planner_params.hpp"

using namespace trajectory_frenet;

cv::Point2i offset(float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
}

Eigen::Vector2d rotation(double theta, double x, double y) {
	Eigen::Matrix2d rotationZ;
	rotationZ << std::cos(theta), -1*std::sin(theta),
							 std::sin(theta), std::cos(theta);
	Eigen::Vector2d point;
	point << x, y;
	return rotationZ*point;
}

int main(int argc, char** argv) {
    // 1) Construct the planner
    TrajectoryPlanner planner;

    // 2) Build center lane 
    std::vector<std::vector<double>> center_lane;
    const double a = 0.051,    
                b = 3.45,     
                c = 2.05;     
    double distance_traced = 0.0, prevX = 0.0;
    double prevY = 6.2*(std::sin(std::pow((prevX*a - b),2) + (prevX*a - b) + c) + 1);  


    for (double x = 0; x < 155; x += 0.01) {  
        
      double y   = 6.2*(std::sin(std::pow((x*a - b),2) + (x*a - b) + c) + 1);
      double dy  = 6.2*std::cos(std::pow(x*a - b,2) + (x*a - b) + c)*((2*a*(x*a - b)) + a);
      double ddy = 6.2*((-std::sin(std::pow(x*a - b,2) + (x*a - b) + c)*(std::pow((2*a*(x*a - b) + a),2)))
                + (std::cos(std::pow(x*a - b,2) + (x*a - b) + c)*(2*a*a)));
      double curvature = std::abs(ddy) / std::pow(1 + dy*dy, 1.5);
      double yaw = dy;
      distance_traced += std::hypot(x - prevX, y - prevY);
      center_lane.push_back({ x, y, yaw, curvature, distance_traced });
      prevX = x;  prevY = y;
    }

    cv::namedWindow("Trajectory Planner", cv::WINDOW_NORMAL);

    auto y_base = [&](double x){
        return 6.2 * (std::sin(std::pow((x*a - b),2)
                          + (x*a - b)
                          + c) + 1);
    };

    std::vector<std::vector<double>> obstacles = {
        // 3 on-lane (for reference)
        {  30.0, y_base(30.0) },
        {  75.0, y_base(75.0) },
        { 120.0, y_base(120.0) },
        { 130.0, y_base(130.0) },

        // 6 off-lane (bigger lateral offsets)
        {  20.0, y_base(20.0) + 2.5 },
        {  50.0, y_base(50.0) - 3.0 },
        {  90.0, y_base(90.0) + 2.0 },
        { 140.0, y_base(140.0) - 2.2 },
        {  60.0, y_base(60.0) + 3.5 },
        { 110.0, y_base(110.0) - 2.8 }
    };

    // 4) Initial Frenet state
    SamplingParams sp;
    double d0 = -sp.lane_width;  // or use your SamplingParams directly
    double dv0 = 0.0; 
    double da0 = 0.0;
    double s0 = 0.0; 
    double sv0 = 10.0/3.6;
    
    LateralState  lat{d0, dv0, da0};
    LongitudinalState lon{s0, sv0};
    
    double x = 0.0;
	  double y = 0.0;


    // 5) Main planning loop
    int alpha = 0;
    while (true) {
        std::vector<FrenetPath> allPaths;
        FrenetPath p = planner.plan(lat, lon, center_lane, obstacles, allPaths);

        // advance along the chosen path
        d0  = p.d[1][0];  
        dv0 = p.d[1][1];  
        da0 = p.d[1][2];
        s0  = p.s[1][0];  
        sv0 = p.s[1][1];

        lat = {d0, dv0, da0};
        lon = {s0, sv0};
        
        x = p.world[1][0];
        y = p.world[1][1];

        // stopping condition: within 1m of end of centerLane
        // auto& last = center_lane.back();
        if (std::sqrt(std::pow(center_lane[10000][0]-x,2)+std::pow(center_lane[10000][1]-y,2))<=1) {
          break;
        }

        // visualize
        cv::Mat lane(5000, 12500, CV_8UC3, cv::Scalar(255, 255, 255));
        
        // Lane
        for(int i{1}; i<center_lane.size(); i++){
          cv::line(lane, offset(center_lane[i-1][0], center_lane[i-1][1], lane.cols, lane.rows), offset(center_lane[i][0], center_lane[i][1], lane.cols, lane.rows), cv::Scalar(0, 0, 0), 10);
        }

        // Obstacles
        for(int i{0}; i<obstacles.size(); i++){
          cv::circle(lane, offset(obstacles[i][0], obstacles[i][1], lane.cols, lane.rows), 40, cv::Scalar(0, 0, 255), 50);
        }

        // Robot
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 30);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), 
                        lane.cols, lane.rows),     
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 30);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 30);
        cv::line(lane, 
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,0.75)(1), 
                        lane.cols, lane.rows),
                 offset(p.world[0][0]+rotation(p.world[0][2],-1.5,-0.75)(0), p.world[0][1]+rotation(p.world[0][2],-1.5,-0.75)(1), 
                        lane.cols, lane.rows), 
                 cv::Scalar(0, 0, 0), 30);

        // All Trajectories
        for(FrenetPath &t:allPaths){
          for(int i{1}; i<t.world.size(); i++){
            cv::line(lane, offset(t.world[i-1][0], t.world[i-1][1], lane.cols, lane.rows), offset(t.world[i][0], t.world[i][1], lane.cols, lane.rows), cv::Scalar(255, 0, 0), 2);
          }
        }

        // Trajectory
        for(int i{1}; i<p.world.size(); i++){
          cv::line(lane, offset(p.world[i-1][0], p.world[i-1][1], lane.cols, lane.rows), offset(p.world[i][0], p.world[i][1], lane.cols, lane.rows), cv::Scalar(0, 255, 0), 30);
        }

        cv::resizeWindow("Trajectory Planner", 1050, 400);
        cv::imshow("Trajectory Planner", lane);
        cv::waitKey(1);
    }

    return 0;
}
