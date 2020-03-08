#ifndef POSEPREDICTION_HH__
#define POSEPREDICTION_HH__

#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <string>

using void_ptr = std::unique_ptr<void, std::function<void(void*)>>;
using namespace std::vector;
/*
Usage:
    void* thing;
    void_ptr wrapped_thing = {thing, [](void* thing) {
        // destructor goes here.
    }}
    // wrapped_thing.get() returns underlying thing.
*/

class predict_pose {
public:
	push_imu_measurement();
	push_pose_measurement(position vector<float>, rotation vector<float>);

    // Get the pose based on the latest IMU measurement
	get_pose();
    // Get the pose some time in the future 
	get_pose(double milliseconds);



private:
	vector<float> latest_pose;
    vector<float> lastest_rotation;
};

#endif