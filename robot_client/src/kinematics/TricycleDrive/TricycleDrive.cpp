/**
 * @file TricycleDrive.cpp
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "kinematics/TricycleDrive/TricycleDrive.h"

TricycleDrive::TricycleDrive() {

}

TricycleDrive::TricycleDrive(std::shared_ptr<MotorClient> *m, float base, float radius, float ratio) {
    motors = *m;
    setParams(base, radius, ratio);
}

TricycleDrive::~TricycleDrive() {

}

DriveFeedback TricycleDrive::getFeedback() {
    prevFeedback.previous_feedback_time = std::chrono::system_clock::now();
    std::vector<int32_t> ticks = motors->getEncodersData();
    std::vector<float> rpm = motors->getRPMData();
    Feedback.Vel = rpm[0] *RPM_TO_RADS/GEAR_RATIO;
    Feedback.Ticks = ticks[0];
    return Feedback;
}


void TricycleDrive::setParams(float base, float radius, float ratio) {
    wheelBase = base;
    wheelRadius = radius;
    GEAR_RATIO = ratio;
    MS_TO_RPM = GEAR_RATIO * 60 / (2 * M_PI * wheelRadius); //
    ODOM_FACTOR = (2 * M_PI * wheelRadius) / (GEAR_RATIO * motors->getEncoderResolution());
}


void TricycleDrive::setVelocity(float v, float w) {
    float Vel = v ;     // unit: m/s
    Vel *= MS_TO_RPM;
}

/**
 * @brief reset pose to (0,0,0)
 * 
 */
bool TricycleDrive::resetOdom() {
    setOdom({0,0,0});
    return true;
}

/**
 * @brief Set the Odom object
 * 
 * @param newPose 
 */
bool TricycleDrive::setOdom(Pose2D newPose) {
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
Pose2D TricycleDrive::getOdom() {
    return pose;
}

/**
 * @brief Get the Stamped Odom object
 * 
 * @return PoseStamped2D 
 */
PoseStamped2D TricycleDrive::getStampedOdom() {
    return poseStamped;
}

/**
 * @brief 
 * 
 * @param Ticks Ticks 
 * @return Pose2D 
 */
Pose2D TricycleDrive::updateOdom(int32_t Ticks) {
	float delta_distance = (Ticks - Feedback.Ticks)/ODOM_FACTOR;
    Feedback.Ticks = Ticks;
    delta_time_ = std::chrono::duration_cast<std::chrono::microseconds>(Feedback.current_feedback_time -Feedback.previous_feedback_time).count();
    float steering_angle;

    pose.Vx = (cos(steering_angle)*delta_distance)/delta_time_;
    pose.Vy = (-sin(steering_angle)*delta_distance)/delta_time_;
    pose.Vw = steering_angle/delta_time_;
    }

/**
 * @brief 
 * 
 * @return Pose2D 
 */
Pose2D TricycleDrive::calculateOdom() {
    getFeedback();
    return updateOdom(Ticks);
}
