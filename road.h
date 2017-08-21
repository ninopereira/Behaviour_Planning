#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
public:

    double m_update_width = 70;

    string m_ego_rep = " *** ";

    int m_ego_key = -1;

    int m_num_lanes;

    vector<double> m_lane_speeds;

    double m_speed_limit;

    double m_density;

    double m_camera_center;

    map<int, Vehicle> m_vehicles;

    int m_vehicles_added = 0;

    /**
  	* Constructor
  	*/
    Road(double speed_limit, double traffic_density, vector<double> lane_speeds);

  	/**
  	* Destructor
  	*/
  	virtual ~Road();

  	Vehicle get_ego();

  	void populate_traffic();

  	void advance();

  	void display(int timestep);

    void add_ego(int lane_num, double s, vector<double> config_data);

  	void cull();

};

