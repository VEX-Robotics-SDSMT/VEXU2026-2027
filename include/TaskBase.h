#pragma once

#include "pros/rtos.hpp"

namespace Mines
{

class TaskBase
{
    public:
        TaskBase(std::string name, uint32_t stackSize = TASK_STACK_DEPTH_DEFAULT, uint8_t priority = TASK_PRIORITY_DEFAULT);
        virtual void run() = 0;    
    private:
        static void taskFunction(void* param);
        pros::Task m_task;
};

}