#ifndef PREDICTPOSE_HH__
#define PREDICTPOSE_HH__

#include <vector>
#include <chrono>
#include "kalman.hh"

// Copied over from Spindle data_format.hh

// Designed to be a drop-in replacement for
// the XrQuaternionf datatype. 
struct quaternion_t {
	float x,y,z,w;
};

// Designed to be a drop-in replacement for
// the XrVector3f datatype. 
struct vector3_t {
	float x,y,z;
};

// Designed to be a drop-in replacement for
// the XrPosef datatype. 
struct pose_t {
	quaternion_t orientation;
	vector3_t position;
};

// A particular pose, sampled at a particular point in time.
struct pose_sample {
	pose_t pose;
	std::chrono::time_point<std::chrono::system_clock> sample_time;
};

// Im assuming that the IMU will feed data in this format
struct imu_measurement {
    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    std::chrono::time_point<std::chrono::system_clock> sample_time;
};

class predict_pose {
    public:
        predict_pose(pose_sample initial_pose) : _latest_pose(initial_pose) {};
        void push_imu_measurement(imu_measurement);
        void push_pose_measurement(pose_sample);
        pose_t get_pose();
        pose_t get_pose(float ms_ahead);
    private:
        pose_t _calculate_pose(float time_interval);
    private:
        pose_sample _latest_pose;
        kalman_filter* _filter;
        std::vector<float> _latest_vel;
        std::vector<float> _latest_acc;
        std::vector<float> _latest_gyro;
};

#endif