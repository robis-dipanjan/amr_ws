/**
 * @file DifferentialDrive.h
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <math.h>
#include <memory>
#include "KincoMotor.h"
#include "moonsMotor.h"
#include "DataStructures.h"


typedef struct {
    float lVel=0.0;
    float rVel=0.0;
    int32_t lTicks;
    int32_t rTicks;
    std::chrono::time_point <std::chrono::system_clock>current_feedback_time, previous_feedback_time;
} DriveFeedback;

class DifferentialDrive {
public:
    DifferentialDrive(std::shared_ptr<MotorClient> motorClient, float base, float radius, float ratio);
    ~DifferentialDrive();

    void setVelocity(float v, float w);
    void setParams(float base, float radius, float ratio);
    DriveFeedback getFeedback();
    bool resetOdom();
    bool setOdom(Pose2D newPose);
    Pose2D getOdom();
    PoseStamped2D getStampedOdom();
    Pose2D updateOdom(int32_t leftTicks, int32_t rightTicks);
    Pose2D calculateOdom();

private:
    double x{0.0};
    double y{0.0};
    double th{0.0};
    double d{0.0};
    Pose2D pose;
    double delta_time_;
    PoseStamped2D poseStamped;
    float wheelBase;
    float wheelRadius;
    float MS_TO_RPM;
    float GEAR_RATIO;
    float RPM_TO_RADS = (2 * M_PI) / 60.0;
    float ODOM_FACTOR;
    int motorSelection {0};
    DriveFeedback dFeedback;
    DriveFeedback prevFeedback;
    DriveFeedback currFeedback;
    std::shared_ptr<MotorClient> motors;
    std::chrono::system_clock::time_point previous_feedback_time_;

};