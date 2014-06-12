#include <box_detection_and_shoring/shoring_node.h>
#include <iostream>
#include <stdexcept>

namespace box_detection_and_shoring
{

ShoringNode::ShoringNode()
{
}

}




int main(int argc, char ** argv)
{
    ros::init(argc, argv, "shoring");

    try
    {
        box_detection_and_shoring::ShoringNode node;
        ros::spin();
        return 0;
    }
    catch (std::exception & ex)
    {
        std::cerr << "Unhandled exception: " << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unhandled exception" << std::endl;
        return 1;
    }
}
