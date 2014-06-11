#include <baxter_open_door_control/baxter_open_door_control.h>

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc,argv,"baxter_open_door_control");

        baxter_open_door_control::BaxterOpenDoorControl baxter_control;

        baxter_control.init();

        ros::spin();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
