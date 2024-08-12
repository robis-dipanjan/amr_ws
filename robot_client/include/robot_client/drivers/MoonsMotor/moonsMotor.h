#pragma once

#include <string>
#include <iostream>
#include "modbus/modbus.h"
#include "drivers/MotorClient.h"

class MoonsMotorClient: public MotorClient{
    public:
        MoonsMotorClient(const std::string &host, int port);
        ~MoonsMotorClient();
        bool connection_status = false;
        void write_registers(int addr,int nb,uint16_t write_data[]);
        void write_read_register(int write_add, int write_nb, uint16_t write_data[], 
                                    int read_add, int read_nb, uint16_t read_data[]);
        void read_register(int startAddress, int offset, uint16_t read_data[]);
        bool connect_comm();
        bool is_connected();
        bool disconnect();
        bool setVelocities(uint16_t leftVel, uint16_t rightVel);

        void setVelocitiesRPM(std::vector<float> vels) override;
        float getEncoderResolution() override;
        std::vector<float> getRPMData() override;
        std::vector<int32_t> getEncodersData() override;
        void receiveMsg(const char * msg, size_t msgSize) override;
        bool initializeMotor() override;
        bool faultStates() override;
        void stopAll() override;
        
    private:
        modbus_t *plc;
        uint16_t write_data[2] = {0};
        uint16_t read_data[2] = {0};
        uint16_t write_left_data[2] {0};
        uint16_t write_right_data[2] {0};
        uint16_t left_RPM_data[2] {0};
        uint16_t right_RPM_data[2] {0};
        int leftRPMAddr = 7;
        int rightRPMAddr = 8;
        int leftEncoderAddr = 5;
        int rightEncoderAddr = 6;
        uint16_t left_encoder_data[2] {0};
        uint16_t right_encoder_data[2] {0};
        uint16_t fault_States_data[8];
        int status;
        int startAddress;
        int offset;
        uint32_t encoderResolution = 10000;
        std::vector<int> faultStateAddr {29, 30, 31, 32, 33, 34, 35, 36}; //Subject to change
};