#include <iostream>
#include "utils/xmlParser.h"

bool xmlParser::configFile(){
    
}


int main(){
    tinyxml2::XMLDocument document;
    tinyxml2::XMLError loadError = document.LoadFile("config/Config.xml");
    if(loadError!= tinyxml2::XML_SUCCESS){
        std::cout << "Reading from config file failed" << std::endl;
    }
    else{
        tinyxml2::XMLNode *root = document.FirstChildElement("Config");

        if (root!=nullptr){
            tinyxml2::XMLElement *motorsPathElement = root->FirstChildElement("MotorsXmlPath");
            tinyxml2::XMLElement *kinematicsPathElement = root->FirstChildElement("KinematicsXmlPath");

            const char *motorsPath = motorsPathElement->GetText();
            const char *KinematicsPath = kinematicsPathElement->GetText();
            
            std::cout << "MotorsXmlPath: " << motorsPath << std::endl;
            std::cout << "KinematicsXmlPath: " << KinematicsPath << std::endl;
        }
        else{
            std::cout << "Configurations not declared" << std::endl;
        }
    }
    return 0;
}