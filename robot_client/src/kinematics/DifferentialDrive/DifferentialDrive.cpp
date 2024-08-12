/**
 * @file DifferentialDrive.cpp
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "DifferentialDrive.h"

DifferentialDrive::DifferentialDrive(std::shared_ptr<MotorClient> motorClient, float base, float radius, float ratio) {
    if (motorClient) {
        motors = motorClient;
    } else {
        throw std::invalid_argument("Motor client is null.");
    }
    setParams(base, radius, ratio);
}

DifferentialDrive::~DifferentialDrive() {

}

DriveFeedback DifferentialDrive::getFeedback() {
    dFeedback.current_feedback_time = std::chrono::system_clock::now();
    std::vector<int32_t> ticks = motors->getEncodersData();
    std::vector<float> rpm = motors->getRPMData();
    dFeedback.lVel = rpm[0] * RPM_TO_RADS/GEAR_RATIO;
    dFeedback.rVel = rpm[1] * RPM_TO_RADS/GEAR_RATIO;
    dFeedback.lTicks = ticks[0];
    dFeedback.rTicks = ticks[1];
    return dFeedback;
}


void DifferentialDrive::setParams(float base, float radius, float ratio) {
    wheelBase = base;
    wheelRadius = radius;
    GEAR_RATIO = ratio;
    MS_TO_RPM = GEAR_RATIO * 60 / (2 * M_PI * wheelRadius); //
    ODOM_FACTOR = (2 * M_PI * wheelRadius) / (GEAR_RATIO * motors->getEncoderResolution());
}

void DifferentialDrive::setVelocity(float v, float w) {
    float rVel = v + w * wheelBase / 2;     // unit: m/s
	float lVel = v - w * wheelBase / 2;

    rVel *= MS_TO_RPM;
    lVel *= MS_TO_RPM;

    motors->setVelocitiesRPM({lVel, rVel});
}

/**
 * @brief reset pose to (0,0,0)
 * 
 */
bool DifferentialDrive::resetOdom() {
    setOdom({0,0,0});
    return true;
}

/**
 * @brief Set the Odom object
 * 
 * @param newPose 
 */
bool DifferentialDrive::setOdom(Pose2D newPose) {
    pose = newPose;
    poseStamped.pose = newPose;
    poseStamped.time = std::chrono::system_clock::now();
    return true;
}

/**
 * @brief Get the Odom object
 * 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::getOdom() {
    return pose;
}

/**
 * @brief Get the Stamped Odom object
 * 
 * @return PoseStamped2D 
 */
PoseStamped2D DifferentialDrive::getStampedOdom() {
    return poseStamped;
}

/**
 * @brief 
 * 
 * @param leftTicks 
 * @param rightTicks 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::updateOdom(int32_t leftTicks, int32_t rightTicks) {
	float d_left = (leftTicks - prevFeedback.lTicks) * ODOM_FACTOR;
	float d_right = (rightTicks - prevFeedback.rTicks) * ODOM_FACTOR;
    prevFeedback = dFeedback;
    delta_time_ = std::chrono::duration_cast<std::chrono::microseconds>(dFeedback.current_feedback_time -dFeedback.previous_feedback_time).count();
    double x{0.0};
    double y{0.0};
    double th{0.0};
    double d{0.0};    
    double dt = delta_time_/1000000;
	d = (d_right + d_left) / 2;
	th = (d_right - d_left) / wheelBase;

	if (d != 0) {        
		x = cos(th) * d;
		y = -sin(th) * d;
        
        pose.Vx = x/dt;
        if (pose.Vx < 0.000001){
            pose.Vx = 0.0;
        }
        pose.Vy = y/dt;
        pose.Vw = th/dt;

		pose.x += (cos(pose.theta) * x - sin(pose.theta) * y);
		pose.y += (sin(pose.theta) * x + cos(pose.theta) * y);
	}
    
	pose.theta += th;
    poseStamped.pose = pose;
    poseStamped.time = std::chrono::system_clock::now();
    dFeedback.previous_feedback_time = dFeedback.current_feedback_time;

    return pose;
}

/**
 * @brief 
 * 
 * @return Pose2D 
 */
Pose2D DifferentialDrive::calculateOdom() {
    getFeedback();
    return updateOdom(dFeedback.lTicks, dFeedback.rTicks);
}
