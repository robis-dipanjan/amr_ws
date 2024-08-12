#pragma once
#include <string>
#include <tinyxml2.h>


class xmlParser{
    public:
        xmlParser();
        bool configFile();
        std::string motorpath;
        std::string kinematicsPath;
        float wheelBase;
        float wheelRadius;
        float gearRatio;
        int MotorMake;
        int KinematicsType;
    private:
};

