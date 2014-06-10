#ifndef BOX_DETECTION__BOX_DETECTION_H_
#define BOX_DETECTION__BOX_DETECTION_H_

#include <ros/node_handle.h>
#include <tf/transform_listener.h>

namespace box_detection
{

class BoxDetectionNode
{
public:
    BoxDetectionNode();

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    ros::Publisher pub_cloud_plane;
};

}

#endif // BOX_DETECTION__BOX_DETECTION_H_
