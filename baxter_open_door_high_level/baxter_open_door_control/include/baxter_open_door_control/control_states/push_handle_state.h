#ifndef push_handle_state_h___
#define push_handle_state_h___

#include <baxter_open_door_control/control_states/baxter_open_door_control_state.h>

namespace baxter_open_door_control
{

class PushHandleState : public BaxterOpenDoorControlState
{
public:
    PushHandleState(BaxterOpenDoorControl* baxter_open_door_control);

    virtual ~PushHandleState();

    virtual std::string getStateName()
    {
        return std::string("Push Handle State");
    }

    virtual bool setDetectMode();

    virtual bool setGraspHandleMode();

    virtual bool setPushHandleMode();

    virtual bool setOpenDoorMode();

    virtual bool setIdleMode();

    virtual void timedExecution();


};

}

#endif
