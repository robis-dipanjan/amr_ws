/**
 * @file DataStructures.h
 * @brief This is the header file for storing common DS used through out AccioPickPilot
 *
 * @copyright Copyright (c) Accio Robotics
 *
 * @author Jacob V Sanoj
 * @date 28-11-2023
 */
#ifndef INCLUDE_DATASTRUCTURES_H
#define INCLUDE_DATASTRUCTURES_H

// Can use signals 10, 12 and 37-61
#define SIGLOWBATT 10
#define SIGSUFFBATT 37
#define SIGCHARGED 12
#define SIGROBOTERROR 38
#define SIGREPLAN 39
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <map>
#include <csignal>
#include <chrono>
#include "DataStructures.h"

/**
 * @brief enum for battery status
 *
 */

enum BatteryStatus
{
    CRITICAL,
    NORMAL,
    BATT_CHARGING,
    BATT_CHARGED
};
// When updating any of the enums above, make sure to update the batteryStatusEnumVector correctly indexed.
inline std::vector<std::string> batteryStatusEnumVector = {"CRITICAL", "NORMAL", "BATT_CHARGING", "BATT_CHARGED"};
/**
 * @struct Battery status
 *
 * @brief Struct to hold the battery info of each robot
 *.
 */
struct Battery
{
    float batteryCharge{100.0}, batteryVoltage{48.0}, batteryHealth{100.0}, reach{1000.0};
    bool charging{false};
    BatteryStatus status{BatteryStatus::NORMAL};

    Battery(){};

    Battery(float batteryCharge, float batteryVoltage, float batteryHealth, float reach, bool charging)
    {
        this->batteryCharge = batteryCharge;
        this->batteryVoltage = batteryVoltage;
        this->batteryHealth = batteryHealth;
        this->reach = reach;
        this->charging = charging;
    };
};
struct SafetyState
{
    std::string eStop{"NONE"};
    bool fieldViolation{false};
};
/**
 * @struct Error Status
 *
 * @brief This struct will hold relevant info related to Error states of robots
 *.
 */
struct ErrorStatus
{
    std::string errorType{""}, errorDescription{""}, errorLevel{""};
    std::vector<std::map<std::string, std::string>> errorReference;
};
/**
 * @enum Robot States
 *
 * @brief The possible states the robot can be in
 *.
 */

enum RobotState
{
    INIT,
    ASSIGNED,
    MOVING,
    REACHED,
    WAITING,
    EXTENDING,
    EXTENDED,
    DOCKING,
    DOCKED,
    CHARGING,
    CHARGED,
    UNDOCKING,
    UNDOCKED,
    RETRACTING,
    RETRACTED,
    IDLE,
    FAILED,
    ERROR,
    OBS_STOP,
    INT_STOP,
    EXT_STOP,
    STATE_MAINTENANCE,
    STATE_MANUAL
};

enum ActionStatus
{
    RUNNING,
    FINISHED,
    HALTED,
    FAULT
};
/**
 * @brief  Struct to hold the state node data
 *
 */
struct StateNode
{
    std::string nodeId{""};
    int sequenceId{0};
    bool released{false};
};
/**
 * @brief  Struct to hold the state edge data
 *
 */
struct StateEdge
{
    std::string edgeId{""};
    int sequenceId{0};
    bool released{false};
};
struct ActionState
{
    std::string actionId{""};
    std::string actionStatus{"WAITING"};
};
// When updating any of the enums above, make sure to update the vector correctly indexed.
inline std::vector<std::string> robotStatesEnumVector = {"INIT", "ASSIGNED", "MOVING", "REACHED", "WAITING", "EXTENDING", "EXTENDED", "DOCKING", "DOCKED", "CHARGING", "CHARGED", "UNDOCKING", "UNDOCKED", "RETRACTING", "RETRACTED", "IDLE", "FAILED", "ERROR", "OBS_STOP", "INT_STOP", "EXT_STOP", "STATE_MAINTENANCE", "STATE_MANUAL"};

/**
 * @struct Robot Status
 *
 * @brief This struct will hold relevant info related to the status of the robots
 *.
 */
struct RobotStatus
{
    float batteryPercentage{0.0}, x{0.0}, y{0.0}, z{0.0}, theta{0.0}, distanceLastNode{0.0}, vx{0.0}, vy{0.0}, vz{0.0}, vtheta{0.0};
    std::string lastNodeId{""}, serialNumber{""}, mapId{""};
    bool positionInitialized{true};
    std::vector<StateNode> nodeStates;
    std::vector<std::string> nodeIds;
    std::vector<StateEdge> edgeStates;
    std::vector<std::string> edgeIds;
    std::vector<ActionState> actionStates;
    RobotState state;

    RobotStatus(){};

    // RobotStatus(std::string type, std::string serialNumber)
    // {
    //     this->serialNumber = serialNumber;
    // };
};
/**
 * @enum Actions
 *
 * @brief Actions depending on VDA5050
 *.
 */
enum BlockingType
{
    NONE,
    SOFT,
    HARD
};
// When updating any of the enums above, make sure to update the blockingTypesEnumVector correctly indexed.
inline std::vector<std::string> blockingTypesEnumVector = {"NONE", "SOFT", "HARD"};
/**
 * @brief  Struct to hold the actions
 *
 */
struct Actions
{
    std::string actionType{""}, actionId{""};
    std::vector<std::map<std::string, std::string>> actionParams;
    BlockingType blockingType;
};

struct NodePosition
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    std::string mapId{""};
};

/**
 * @brief  Struct to hold the order data for node
 *
 */
struct OrderNode
{
    std::string nodeId{0};
    int sequenceId{0};
    bool released{false};
    std::vector<Actions> actions;
    NodePosition nodePosition;
};
/**
 * @brief  Struct to hold the order data for edge
 *
 */
struct OrderEdge
{
    std::string edgeId{""}, startNodeId{""}, endNodeId{""};
    int sequenceId{0};
    bool released{false};
    int orientation{0};
    std::string orientationType{"TANGENTIAL"};
    bool rotationAllowed{false};
    std::vector<Actions> actions;
};
/**
 * @brief  Struct to hold the order data
 *
 */
struct Order
{
    std::string orderId{""}, robotId{""}, zoneSetId{""};
    int orderUpdateId{0};
    std::vector<OrderNode> nodes;
    std::vector<OrderEdge> edges;
};

struct Pose3D
{
    double x;
    double y;
    double z;
};

struct Pose2D
{
    double x;
    double y;
    double theta;
    double Vx;
    double Vy;
    double Vw;
};
struct PoseStamped2D
{
    std::chrono::system_clock::time_point time;
    Pose2D pose;
};

struct Velocity
{
    std::chrono::system_clock::time_point time;
    double leftOmega=0.0;
    double rightOmega=0.0;
    double robotV=0.0;
    double robotOmega=0.0;
};

struct CameraStampedData
{
   std::chrono::system_clock::time_point time;
   int tagNodeId;
   Pose2D CameraPose;
   bool valid;
};

// typedef struct pos3D
// {
//     float x;
//     float y;
//     float z;
// } pos3D;

struct Euler
{
    double roll;
    double pitch;
    double yaw;
};

struct IMUData
{
    std::chrono::system_clock::time_point time;
    int counter;
    int status;
    Euler orientation;
    Pose3D accel;
    Pose3D gyro;
    Pose3D mag;
};

struct TagData
{
   std::chrono::system_clock::time_point time;
   bool valid;
   bool isDmTag;
   int nodeId;
   Pose2D pose;
};

struct ActionParams
{
    double difference;
    std::string direction;
};

struct Task
{
    std::string taskType{""};
    std::string subTaskType{""};
    bool isNodeTask{true};
    std::vector<OrderNode> taskNodes;                            
    std::vector<OrderEdge> taskEdges;
    
};

#endif // INCLUDE_DATASTRUCTURES_H