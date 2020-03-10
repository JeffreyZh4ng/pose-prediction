#include <math.h> 
#include <Eigen/Dense>
#include "predict_pose.hh"

predict_pose::predict_pose(pose_sample initial_pose) {
    _latest_pose = initial_pose;
    _filter = new kalman_filter::kalman_filter(std::chrono::system_clock::now());
}

// Spindle will call this when the IMU reads a new measurement. This function assumes that all the
// cleanup for the IMU values have already happened (removing gravity force component etc)
void predict_pose::push_imu_measurement(imu_measurement new_measurement) {
    _latest_acc = std::vector<float>{new_measurement.ax, new_measurement.ay, new_measurement.az};
    _latest_gyro = _filter->predict_values(new_measurement);

    float time_difference = std::chrono::duration_cast<std::chrono::milliseconds>
            (new_measurement.sample_time - _latest_pose.sample_time).count();

    _latest_pose = {_calculate_pose(time_difference), new_measurement.sample_time};
}

// Sets the new pose from SLAM and assumes a velocity measurement from the previous pose
void predict_pose::push_pose_measurement(pose_sample fresh_pose) {
    float time_difference = std::chrono::duration_cast<std::chrono::milliseconds>
            (fresh_pose.sample_time - _latest_pose.sample_time).count();

    // Update the velocity with the pose position difference over time. This is done because
    // Accelerometer readings have lots of noise and will lead to bad dead reckoning
    _latest_vel[0] = (fresh_pose.pose.position.x - _latest_pose.pose.position.x) / time_difference;
    _latest_vel[1] = (fresh_pose.pose.position.y - _latest_pose.pose.position.y) / time_difference;
    _latest_vel[2] = (fresh_pose.pose.position.z - _latest_pose.pose.position.z) / time_difference;

    _latest_pose = fresh_pose;
}

// Gets the pose based off of the latest IMU reading
pose_t predict_pose::get_pose() {
    return _latest_pose.pose;
}

// Will predict the pose some time in the future by ms_ahead milliseconds
pose_t predict_pose::get_pose(float ms_ahead) {
    return _calculate_pose(ms_ahead);
}

// Helper that does all the math to calculate the poses
pose_t predict_pose::_calculate_pose(float time_interval) {

    // Do some Euler to Quaternion conversion and find the predicted angle 
    Eigen::Quaternion orientation_quaternion = Eigen::Quaternion(
            _latest_pose.pose.orientation.w,
            _latest_pose.pose.orientation.x,
            _latest_pose.pose.orientation.y,
            _latest_pose.pose.orientation.z); 

    Eigen::Vector3f orientation_euler = orientation_quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    orientation_euler(0) += _latest_gyro[0] * time_interval;
    orientation_euler(1) += _latest_gyro[1] * time_interval;
    orientation_euler(2) += _latest_gyro[2] * time_interval;

    orientation_quaternion = Eigen::AngleAxisf(orientation_euler(0), Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(orientation_euler(1), Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(orientation_euler(2), Eigen::Vector3f::UnitZ());

    Eigen::Vector4f coeffs = orientation_quaternion.coeffs();                
    quaternion_t predicted_orientation = {
        coeffs(1),
        coeffs(2),
        coeffs(3),
        coeffs(0),
    };
	vector3_t predicted_position = {
        0.5 * _latest_acc[0] * pow(time_interval, 2) + _latest_vel[0] * time_interval + _latest_pose.pose.position.x,
        0.5 * _latest_acc[1] * pow(time_interval, 2) + _latest_vel[1] * time_interval + _latest_pose.pose.position.y,
        0.5 * _latest_acc[2] * pow(time_interval, 2) + _latest_vel[2] * time_interval + _latest_pose.pose.position.z
    };

    return pose_t{predicted_orientation, predicted_position};
}