/**
 * @file DeviceManager.h
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "drivers/kincoMotor/KincoMotor.h"
#include "drivers/MoonsMotor/moonsMotor.h"
#include "kinematics/DifferentialDrive/DifferentialDrive.h"

class DeviceManager {
    public:
        DeviceManager();
        ~DeviceManager();

        std::shared_ptr<KincoMotorClient> kincoMotor;

        std::shared_ptr<MoonsMotorClient> moonsMotor;

        std::shared_ptr<DifferentialDrive> diffDrive;

        std::shared_ptr<MotorClient> motorClient;

        bool Kinco;
    };