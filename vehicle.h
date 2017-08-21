#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

struct Snapshot {
   int lane;
   double s;
   double v;
   double a;
   std::string state;
   };

using Full_Trajectory = std::vector <Snapshot>;

using Vehicle_ID = int;
using Unique_ID = const int;
using Distance = double;
using Lane = int;
using Position = std::pair<Distance, Lane>;
using Trajectory = std::vector <Position>;
using Prediction = std::pair<Vehicle_ID,Trajectory>;
using Predictions = std::map<Unique_ID,Prediction>; // using map as each vehicle has its own id

using State = std::string;
using Cost = double;

struct TrajectoryData {
       int proposed_lane;
       double avg_speed;
       double max_acceleration;
       double rms_acceleration;
       double closest_approach;
       double end_distance_to_goal;
       int end_lanes_from_goal;
       std::pair<bool,int> collides;
       };

//- predictions
//A dictionary. The keys are ids of other vehicles and the values are arrays
//where each entry corresponds to the vehicle's predicted location at the
//corresponding timestep. The FIRST element in the array gives the vehicle's
//current position. Example (showing a car with id 3 moving at 2 m/s):

//{
//  3 : [
//    {"s" : 4, "lane": 0},
//    {"s" : 6, "lane": 0},
//    {"s" : 8, "lane": 0},
//    {"s" : 10, "lane": 0},
//  ]
//  4 : [
//    {"s" : 3, "lane": 1},
//    {"s" : 6, "lane": 1},
//    {"s" : 9, "lane": 1},
//    {"s" : 12, "lane": 1},
//  ]
//}


class Vehicle {
public:

  struct collider{
    bool collision ; // is there a collision?
    double time; // time collision happens
  };

  int m_L = 1;// used to compare if vehicles in the same lane

  double m_preferred_buffer = 6; // impacts "keep lane" behavior.

  int m_lane;

  double m_s;

  double m_v;

  double m_a;

  double m_target_speed;

  int m_lanes_available;

  double m_max_acceleration;

  int m_goal_lane;

  double m_goal_s;

  string m_state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(Predictions predictions);

  void configure(vector<double> road_data);

  std::string display() const;

  void increment(double dt);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, double at_time);

  collider will_collide_with(Vehicle other, int num_timesteps);

  void realize_state(Predictions predictions);

  void realize_constant_speed();

  double max_accel_for_lane(Predictions predictions, int lane, double s);

  void realize_keep_lane(Predictions predictions);

  void realize_lane_change(Predictions predictions, std::string direction);

  void realize_prep_lane_change(Predictions predictions, std::string direction);

  Trajectory generate_trajectory(int horizon = 10);
  
  Position position_at(double time_t);

  Full_Trajectory trajectory_for_state(State& state, Predictions predictions, int horizon=5);

  Snapshot TakeSnapshot() const;
  //bool compare(std::pair<std::string,double> i, std::pair<std::string,double> j) ;

  void restore_state_from_snapshot(Snapshot snapshot);

  std::string get_next_state(Predictions predictions);

};

#endif
