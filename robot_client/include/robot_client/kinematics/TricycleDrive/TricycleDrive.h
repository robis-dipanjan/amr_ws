/**
 * @file TricycleDrive.h
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-7
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <math.h>
#include <memory>
#include "DataStructures.h"
#include "drivers/MotorClient.h"


typedef struct {
    float Vel=0.0;
    int32_t Ticks;
    std::chrono::time_point <std::chrono::system_clock>current_feedback_time, previous_feedback_time;
} DriveFeedback;

class TricycleDrive {
public:
    TricycleDrive(std::shared_ptr<MotorClient> *m, float base, float radius, float ratio);
    TricycleDrive();
    ~TricycleDrive();

    void setVelocity(float v, float w);
    void setParams(float base, float radius, float ratio);
    DriveFeedback getFeedback();
    bool resetOdom();
    bool setOdom(Pose2D newPose);
    Pose2D getOdom();
    PoseStamped2D getStampedOdom();
    Pose2D updateOdom(int32_t Ticks);
    Pose2D calculateOdom();

private:
    int32_t Ticks;
    Pose2D pose;
    double delta_time_;
    PoseStamped2D poseStamped;
    float wheelBase;
    float wheelRadius;
    float MS_TO_RPM;
    float GEAR_RATIO;
    float RPM_TO_RADS = (2 * M_PI) / 60.0;
    float ODOM_FACTOR;
    DriveFeedback prevFeedback;
    DriveFeedback currFeedback;
    DriveFeedback Feedback;
    std::shared_ptr<MotorClient> motors;
};