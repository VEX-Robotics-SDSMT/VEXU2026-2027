#include "TaskBase.h"

namespace Mines
{
TaskBase::TaskBase(std::string name, uint32_t stackSize, uint8_t priority)
    : m_task(TaskBase::taskFunction, this, priority, stackSize, name.c_str())
{

}

void TaskBase::taskFunction(void* param)
{
    TaskBase* task = static_cast<TaskBase*>(param);
    task->run();
}

};