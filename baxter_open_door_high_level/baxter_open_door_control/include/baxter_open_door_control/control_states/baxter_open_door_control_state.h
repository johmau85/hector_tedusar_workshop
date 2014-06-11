
#ifndef baxter_open_door_control_state_h___
#define baxter_open_door_control_state_h___

#include <boost/smart_ptr.hpp>
#include <ros/ros.h>

namespace baxter_open_door_control
{

class BaxterOpenDoorControl;

class BaxterOpenDoorControlState
{

protected:
    BaxterOpenDoorControl* baxter_open_door_control_;

    // SetRobotOperationMode
    ros::ServiceClient baxter_open_door_mode_client_;

public:
    BaxterOpenDoorControlState(BaxterOpenDoorControl* baxter_open_door_control) :
        baxter_open_door_control_(baxter_open_door_control)
    {
    }

    virtual ~BaxterOpenDoorControlState() {}

    virtual std::string getStateName() = 0;

    virtual bool setDetectMode() = 0;

    virtual bool setGraspHandleMode() = 0;

    virtual bool setPushHandleMode() = 0;

    virtual bool setOpenDoorMode() = 0;

    virtual bool setIdleMode() = 0;

    virtual void timedExecution() = 0;

};

typedef boost::shared_ptr<BaxterOpenDoorControlState> BaxterOpenDoorControlStatePtr;

}

#endif
