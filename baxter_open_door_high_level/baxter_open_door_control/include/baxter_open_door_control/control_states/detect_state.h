
#ifndef detect_state_h___
#define detect_state_h___

#include <baxter_open_door_control/control_states/baxter_open_door_control_state.h>

namespace baxter_open_door_control
{

class DetectState : public BaxterOpenDoorControlState
{
public:
    DetectState(BaxterOpenDoorControl* baxter_open_door_control);

    virtual ~DetectState();

    virtual std::string getStateName()
    {
        return std::string("Detect State");
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
