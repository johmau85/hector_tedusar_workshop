
#ifndef idle_state_h___
#define idle_state_h___

#include <baxter_open_door_control/control_states/baxter_open_door_control_state.h>

namespace baxter_open_door_control
{

class IdleState : public BaxterOpenDoorControlState
{
public:
    IdleState(BaxterOpenDoorControl* baxter_open_door_control);

    virtual ~IdleState();

    virtual std::string getStateName()
    {
        return std::string("Idle State");
    }

    virtual bool setDetectMode();

    virtual bool setGraspHandleMode();

    virtual bool setPushHandleMode();

    virtual bool setOpenDoorMode();

    virtual bool setIdleMode();

    virtual void timedExecution();

private:
    bool promptUser(std::string string);
    bool asked_user_;


};

}

#endif

