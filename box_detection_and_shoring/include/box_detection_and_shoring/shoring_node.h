#ifndef BOX_DETECTION_AND_SHORING__SHORING_NODE_H_
#define BOX_DETECTION_AND_SHORING__SHORING_NODE_H_

#include <ros/node_handle.h>

namespace box_detection_and_shoring
{

class ShoringNode
{
public:
    ShoringNode();

private:
    ros::NodeHandle nh_;
};

}

#endif // BOX_DETECTION_AND_SHORING__SHORING_NODE_H_
