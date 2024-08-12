/**
 * @file MoonsMotor.cpp
 * @author dipanjan.maji@motherson.com
 * @brief 
 * @version 0.1
 * @date 2024-08-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "moonsMotor.h"

MoonsMotorClient::MoonsMotorClient(const std::string &host, int port)
{
    plc = modbus_new_tcp(host.c_str(), port);
    if (plc == nullptr) {
        std::cerr << ("Unable to allocate libmodbus context\n");
        return;
    }else{
        std::cout << "MoonsMotorClient connected succesfully " << std::endl;
    }
}

bool MoonsMotorClient::initializeMotor(){
    std::cout << "Motor initialized" << std::endl;
}


MoonsMotorClient::~MoonsMotorClient()
{
    std::cout << "MoonsMotorClient Disconnected" << std::endl;
    disconnect();
}


bool MoonsMotorClient::connect_comm()
{
    if (modbus_connect(plc) == -1) {
        std::cout << "Failed to connect to modbus device!!! "<< std::endl;
        modbus_free(plc);
	    connection_status = false;

    }
    else {
        connection_status = true;
        std::cout << "Connection to modbus device established "<< std::endl;
    }
    return connection_status;
}


void MoonsMotorClient::write_registers(int addr, int nb, uint16_t write_data[])
{
    try {
        status = modbus_write_registers(plc, addr, nb, write_data);
        if (status == -1) {
            std::cout << ("Unable to write register address");
            throw - 1;
        }
    } catch (int e) {
        std::cout << "Unable to write registers address" << status <<std::endl;
    }
}


void MoonsMotorClient::write_read_register(int write_add, int write_nb, uint16_t write_data[], int read_add, int read_nb, uint16_t read_data[]){
	try 
	{
        status = modbus_write_and_read_registers(plc, write_add, write_nb, write_data, read_add, read_nb, read_data);
        if (status == -1){
            std::cout << "Unable to write  reg addr: " << read_add << std::endl;
	    throw -1;
          }

        }   
	catch (int e) 
	{
        std::cout << "modbus error: " << status << std::endl;
    }
}


void MoonsMotorClient::read_register(int startAddress, int offset, uint16_t read_data[])
{
    try {
        status = modbus_read_registers(plc, startAddress, offset, read_data);
        if (status == -1) {
            std::cout << "Unable to read reg addr:" << std::endl;
            throw - 1;
        }

    }
    catch (int e) {
        std::cout << "modbus_error: " << status << std::endl;
    }
}

bool MoonsMotorClient::setVelocities(uint16_t leftVel, uint16_t rightVel){
    write_left_data[0] = leftVel;
    write_right_data[0] = rightVel;
    write_registers(105, 2, write_left_data);
    write_registers(106, 2, write_right_data);
    return true;
}

std::vector<int32_t> MoonsMotorClient::getEncodersData(){
    read_register(leftEncoderAddr, 2, left_encoder_data);
    read_register(rightEncoderAddr, 2, right_encoder_data);
    for (int i=0; i<3; i++){
        std::cout << "Left Encoder Data: " << left_encoder_data[i] << std::endl;
        std::cout << "Right Encoder Data: " << right_encoder_data[i] << std::endl;
    }
}

std::vector<float> MoonsMotorClient::getRPMData(){
    read_register(leftRPMAddr, 2, left_RPM_data);
    read_register(rightRPMAddr, 2, right_RPM_data);
    for (int i=0; i<3; i++){
        std::cout << "Left RPM Data: " << left_RPM_data[i] << std::endl;
        std::cout << "Right RPM Data: " << right_RPM_data[i] << std::endl;
    }
}

bool MoonsMotorClient::faultStates(){
    for (int i=0; i<8; i++){
        read_register(faultStateAddr[i], 2, fault_States_data);
        std::cout << "Right Encoder Data: " << fault_States_data[i] << std::endl;
    }
    return fault_States_data;
}

void MoonsMotorClient::setVelocitiesRPM(std::vector<float> vels){
 
    setVelocities(vels[0], vels[1]);
}

float MoonsMotorClient::getEncoderResolution(){
    return encoderResolution;
}

void MoonsMotorClient::receiveMsg(const char * msg, size_t msgSize){

}


bool MoonsMotorClient::is_connected()
{
    return connection_status;
}

void MoonsMotorClient::stopAll(){
    std::cout << "Stopping All Moons Motor" << std::endl;
}

bool MoonsMotorClient::disconnect()
{
    modbus_close(plc);
    return true;
}