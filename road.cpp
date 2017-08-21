#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(double speed_limit, double traffic_density, vector<double> lane_speeds) {

    m_num_lanes = lane_speeds.size();
    m_lane_speeds = lane_speeds;
    m_speed_limit = speed_limit;
    m_density = traffic_density;
    m_camera_center = m_update_width/2;
}

Road::~Road() {}

Vehicle Road::get_ego() {
	
    return m_vehicles.find(m_ego_key)->second;
}

void Road::populate_traffic() {
	
    double start_s = std::max(m_camera_center - (m_update_width/2), 0.0);
    for (int l = 0; l < m_num_lanes; l++)
	{
        int lane_speed = m_lane_speeds[l];
		bool vehicle_just_added = false;
        for(int s = start_s; s < start_s+m_update_width; s++)
		{
			
			if(vehicle_just_added)
			{
				vehicle_just_added = false;
			}
            if(((double) rand() / (RAND_MAX)) < m_density)
			{
				
                Vehicle vehicle = Vehicle(l,s,lane_speed,0.0);
                vehicle.m_state = "CS";
                m_vehicles_added += 1;
                m_vehicles.insert(std::pair<int,Vehicle>(m_vehicles_added,vehicle));
				vehicle_just_added = true;
			}
		}
	}
	
}

void Road::advance() {
	
    Predictions predictions;

    map<int, Vehicle>::iterator it = m_vehicles.begin();
    int unique_key = 0;
    while(it != m_vehicles.end())
    {
        int v_id = it->first;
        Trajectory trajectory = it->second.generate_trajectory(10);
        Prediction prediction = std::make_pair(v_id,trajectory);
        predictions.insert(std::make_pair(unique_key++,prediction)); // use and then increment unique_key
        ++it;
    }
    it = m_vehicles.begin();
    while(it != m_vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == m_ego_key)
        {
        	it->second.update_state(predictions);
        	it->second.realize_state(predictions);
        }
        it->second.increment(1);
        
        it++;
    }
    
}

void Road::display(int timestep) {

    Vehicle ego = m_vehicles.find(m_ego_key)->second;
    double s = ego.m_s;
    string state = ego.m_state;

    m_camera_center = max(s, m_update_width/2.0);
    int s_min = max(m_camera_center - m_update_width/2.0, 0.0);
    int s_max = s_min + m_update_width;

    vector<vector<string> > road;

    for(int i = 0; i < m_update_width; i++)
    {
    	vector<string> road_lane;
        for(int ln = 0; ln < m_num_lanes; ln++)
    	{
    		road_lane.push_back("     ");
    	}
    	road.push_back(road_lane);

    }

    map<int, Vehicle>::iterator it = m_vehicles.begin();
    while(it != m_vehicles.end())
    {

        int v_id = it->first;
        Vehicle v = it->second;

        if(s_min <= v.m_s && v.m_s < s_max)
        {
        	string marker = "";
            if(v_id == m_ego_key)
        	{
                marker = m_ego_rep;
        	}
        	else
        	{
        		
        		stringstream oss;
        		stringstream buffer;
        		buffer << " ";
        		oss << v_id;
        		for(int buffer_i = oss.str().length(); buffer_i < 3; buffer_i++)
        		{
        		    buffer << "0";
        		
        		}
        		buffer << oss.str() << " ";
        		marker = buffer.str();
        	}
            road[int(v.m_s - s_min)][int(v.m_lane)] = marker;
        }
        it++;
    }
    ostringstream oss;
    oss << "+Meters ======================+ step: " << timestep << endl;
    int i = s_min;
    for(int lj = 0; lj < road.size(); lj++)
    {
        if(i%20 ==0)
    	{
    	    stringstream buffer;
    	    stringstream dis;
        	dis << i;
        	for(int buffer_i = dis.str().length(); buffer_i < 3; buffer_i++)
        	{
        		 buffer << "0";
        	}
    	    
    		oss << buffer.str() << dis.str() << " - ";
    	}
    	else
    	{
    		oss << "      ";
    	}          
    	i++;
    	for(int li = 0; li < road[0].size(); li++)
    	{
    		oss << "|" << road[lj][li];
    	}
    	oss << "|";
    	oss << "\n";
    }
    
    cout << oss.str();

}

void Road::add_ego(int lane_num, double s, vector<double> config_data) {
	
    map<int, Vehicle>::iterator it = m_vehicles.begin();
    while(it != m_vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.m_lane == lane_num && v.m_s == s)
        {
            m_vehicles.erase(v_id);
        }
        it++;
    }
    Vehicle ego = Vehicle(lane_num, s, m_lane_speeds[lane_num], 0);
    ego.configure(config_data);
    ego.m_state = "KL";
    m_vehicles.insert(std::pair<int,Vehicle>(m_ego_key,ego));
    
}

void Road::cull() {
	
	
    Vehicle ego = m_vehicles.find(m_ego_key)->second;
    double center_s = ego.m_s;
    set<vector<double>> claimed;

    map<int, Vehicle>::iterator it = m_vehicles.begin();
    while(it != m_vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        vector<double> claim_pair = {(double)v.m_lane,v.m_s};
        claimed.insert(claim_pair);
        it++;
    }
    it = m_vehicles.begin();
    while(it != m_vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if( (v.m_s > (center_s + m_update_width / 2.0) ) || (v.m_s < (center_s - m_update_width / 2.0) ) )
    	{
    		try {
                claimed.erase({(double)v.m_lane,v.m_s});
    		}
    		catch (const exception& e) {
    			continue;
    		}
            m_vehicles.erase(v_id);

    		bool placed = false;
    		while(!placed) {
                int lane_num = rand() % m_num_lanes;
                int ds = rand() % 14 + (m_update_width/2.0-15);
                if(lane_num > m_num_lanes/2.0)
    			{
    				ds*=-1;
    			}
    			int s = center_s + ds;
                if(claimed.find({(double)lane_num,(double)s}) != claimed.end())
    			{
    				placed = true;
                    int speed = m_lane_speeds[lane_num];
    				Vehicle vehicle = Vehicle(lane_num, s, speed, 0);
                    m_vehicles_added++;
                    m_vehicles.insert(std::pair<int,Vehicle>(m_vehicles_added,vehicle));
                    cout << "adding vehicle "<< m_vehicles_added << " at lane " << lane_num << " with s=" << s << endl;
    			}

    		}
    	}
    	it++;
    }
}



