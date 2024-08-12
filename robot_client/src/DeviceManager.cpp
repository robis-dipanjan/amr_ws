
/**
 * @file DeviceManager.cpp
 * @author your name (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-27
 */


#include "DeviceManager.h"

DeviceManager::DeviceManager() {
    Kinco = true;
    
    if (Kinco) {
        kincoMotor = std::make_shared<KincoMotorClient>("192.168.0.7", 20001);
        kincoMotor->addMotor(0x07, "LEFT");
        kincoMotor->addMotor(0x06, "RIGHT");
        diffDrive = std::make_shared<DifferentialDrive>(kincoMotor, 0.40, 0.10, 15.0);

    } else {
        moonsMotor = std::make_shared<MoonsMotorClient>("192.66.1.4", 502);
        diffDrive = std::make_shared<DifferentialDrive>(moonsMotor, 0.40, 0.10, 15.0);
    }
}

DeviceManager::~DeviceManager() {
    
}