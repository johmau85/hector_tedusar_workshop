
#ifndef grasp_handle_state_h___
#define grasp_handle_state_h___

#include <baxter_open_door_control/control_states/baxter_open_door_control_state.h>

namespace baxter_open_door_control
{

class GraspHandleState : public BaxterOpenDoorControlState
{
public:
    GraspHandleState(BaxterOpenDoorControl* baxter_open_door_control);

    virtual ~GraspHandleState();

    virtual std::string getStateName()
    {
        return std::string("Grasp Handle State");
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
