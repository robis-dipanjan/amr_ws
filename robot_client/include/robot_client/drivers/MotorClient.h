#pragma once

#include "communication/tcp_client.h"

class MotorClient : public TcpClient{
    public:
        MotorClient() = default;
        virtual ~MotorClient() = default;
        virtual void setVelocitiesRPM(std::vector<float> vels) = 0;
        virtual float getEncoderResolution() = 0;
        virtual std::vector<float> getRPMData() =0;
        virtual std::vector<int32_t> getEncodersData() =0;
        virtual void receiveMsg(const char * msg, size_t msgSize) = 0;
        virtual bool initializeMotor() = 0;
        virtual bool faultStates() = 0;
        virtual void stopAll() = 0;
};