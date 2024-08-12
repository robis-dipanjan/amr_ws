/**
 * @file KincoMotor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <string>
#include <vector>
#include <memory>
#include "drivers/MotorClient.h"

typedef struct {
    int32_t position;   //
    int32_t speed;      // rpm, DEC=[(RPM*512*Encoder_Resolution)/1875]
    uint16_t nodeId;    //
    int16_t current;    // 
    std::string name;   //
    uint16_t errorState;
} Motor;

typedef enum {
OPERATIONAL 		=	0x01,
STOP_STATE			=	0x02,
PRE_OPERATIONAL		=	0x80,
RESET_NODE			=	0x81,
RESET_COMMUNICATION	=	0x82,
ERROR_STATE		    =	0xFF,
} CanMode;

typedef enum {
	POWER_OFF = 0x06, POWER_ON = 0x0F, ERROR_RESET=0x86,
} CtrlWord;

typedef enum {
	POSITION_CTRL = 1, SPEED_CTRL = 3, TORQUE_CTRL = 4, QUICK_SPEED_CTRL = -3, MOTOR_OFF = 100,
} OpMode;

typedef enum {
    DRIVER_FAULT   = 0x008,
    EMERGENCY_STOP = 0x0020
} Error;

typedef union {
    char msg[13];
    struct {
        uint8_t dlc;
        uint8_t std;
        uint8_t rtr;
        uint8_t idHigh;
        uint8_t idLow;
        uint8_t data[8];
    };
}CANPacket;

/// @brief 
class KincoMotorClient : public MotorClient {
public:
    KincoMotorClient(const std::string &host, int port);
    ~KincoMotorClient();
private:
    std::string _host;
    int _port;
    std::vector<Motor> motors;
    bool _connected;
    std::mutex mMutex;
    bool errorFound;

    uint32_t encoderResolution = 10000;
    float RPM_FACTOR = (512.0 * encoderResolution) / 1875.0;

    bool sendCANPacket(uint32_t id, uint8_t dlc, uint8_t *TxData);
    void setMode(uint8_t nodeId, OpMode mode);
    void setMotorOff(uint8_t nodeId);
    void setVelocity(uint8_t nodeId, int32_t vel);
    bool setDeviceState(uint8_t nodeId, CanMode state);
    void COReceive(const uint8_t * data, uint8_t len);
    void posSpeedFB(uint8_t nodeId, int32_t motorPos, int32_t motorSpd);
    void currentFB(uint8_t nodeId, int16_t motorCur);
    void errorStates(uint8_t nodeId, uint16_t errorState);
    void resetError(uint8_t nodeId);

public:
    void initAll();
    void stopAll() override;
    void addMotor(int id);
    void addMotor(int id, std::string name);
    void setModes(std::vector<OpMode> modes);
    void setDeviceStates(std::vector<CanMode> modes);
    void setVelocities(std::vector<int32_t> vels);
    void setVelocitiesRPM(std::vector<float> vels);
    std::vector<int32_t> getEncodersData();
    std::vector<float> getRPMData();
    bool resetErrors();
    bool faultStates() override;
    float getEncoderResolution();
    bool initializeMotor() override;
    void receiveMsg(const char * msg, size_t msgSize) override;
};