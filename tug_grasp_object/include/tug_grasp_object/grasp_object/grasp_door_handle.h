#ifndef GRASP_DOOR_HANDLE_
#define GRASP_DOOR_HANDLE_
#include <tug_grasp_object/grasp_object/grasp_object.h>

namespace grasp_object
{

class GraspDoorHandle : public GraspObject
{
public:
    GraspDoorHandle(std::string name, geometry_msgs::PoseStamped pose, std::string arm);
    virtual ~GraspDoorHandle();

    virtual bool graspObject();


};


}

#endif
