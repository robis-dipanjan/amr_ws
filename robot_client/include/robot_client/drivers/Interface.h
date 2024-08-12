/**
 * @file Interface.h
 * @author Dipanjan Maji (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

class Protocol
{
    public:
        virtual ~Protocol() {}
        virtual void connectTo() = 0;
        virtual void receiveMsg() = 0;
        virtual void sendMsg() = 0;
};