#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <utility> // make_pair


using Vehicle_ID = int;
using Unique_ID = const int;
using Distance = double;
using Lane = int;
using Position = std::pair<Distance, Lane>;
using Trajectory = std::vector <Position>;
using Prediction = std::pair<Vehicle_ID,Trajectory>;
using Predictions = std::map<Unique_ID,Prediction>;

//static inline double computeSquare (double x) { return x*x; };
int main()
{
    std::vector<double> accel = {1.1,4.2,-10.3,1.2,7.5,-2.0, 3,-8.1};
    double max_accel = std::fabs(*std::max_element(accel.begin(), accel.end(),[](double lhs, double rhs) {
            return std::fabs(lhs) < std::fabs(rhs);}));
    std::cout << max_accel << std::endl;

    // same as before but with lambda function which doesn't require the inline function declaration above
    std::transform(accel.begin(), accel.end(), accel.begin(), [](double x){return x*x;});

    // compute squares
    for (auto i: accel)
    {
        std::cout << i << ' ';
    }
    std::cout << std::endl;

    Trajectory trajectory;
    trajectory.push_back(std::make_pair<Distance, Lane>(2,0));
    trajectory.push_back(std::make_pair<Distance, Lane>(4,0));
    trajectory.push_back(std::make_pair<Distance, Lane>(6,0));
    const Prediction prediction1 = std::make_pair(1,trajectory);
    const Prediction prediction2 = std::make_pair(2,trajectory);
    const Prediction prediction3 = std::make_pair(3,trajectory);
    Predictions predictions = {
                                        std::make_pair(1,prediction1),
                                        std::make_pair(2,prediction2)
    };
    predictions.insert(std::make_pair(3,prediction3));

    // to access map values we have to use iterators
    for (std::map<Unique_ID,Prediction>::iterator itr = predictions.begin(); itr != predictions.end(); ++itr) {
        auto ptr_prediction = itr->second; // ptr_prediction points to the prediction
        auto ptr_trajectory = ptr_prediction.second; // points to the trajectory
        std::cout << "Distance = " << ptr_trajectory[1].first << std::endl;
    }
}
