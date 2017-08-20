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
using FullTrajectory = std::vector <Snapshot>;

using Vehicle_ID = int;
using Unique_ID = const int;
using Distance = double;
using Lane = int;
using Position = std::pair<Distance, Lane>;
using Trajectory = std::vector <Position>;
using Prediction = std::pair<Vehicle_ID,Trajectory>;
using Predictions = std::map<Unique_ID,Prediction>; // using map as each vehicle has its own id

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
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(map<int, vector <vector<int> > > predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int> > > predictions);

  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  vector<vector<int> > generate_predictions(int horizon);
  
  bool compare(std::pair<std::string,double> i, std::pair<std::string,double> j) ;

  std::string get_next_state(map<int,vector < vector<int> > > predictions);

};

#endif
