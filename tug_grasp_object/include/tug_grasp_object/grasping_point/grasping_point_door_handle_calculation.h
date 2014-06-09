#ifndef GRASPING_POINT_DOOR_HANDLE_
#define GRASPING_POINT_DOOR_HANDLE_

#include <tug_grasp_object/grasping_point/grasping_point_object_calculation.h>

namespace grasping_point
{
static const double RAD2DEG = 57.2957795;

// Grasp axis orientation
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_direction_t {UP, DOWN};
enum grasp_rotation_t {FULL, HALF};

class GraspingPointDoorHandle : public GraspingPointObjectCalculation
{
public:
    GraspingPointDoorHandle();

    virtual ~GraspingPointDoorHandle();

    virtual void calculateGraspingPoints(geometry_msgs::PoseStamped object_pose, GraspData data, std::vector<moveit_msgs::Grasp>& grasps);
private:
    bool generateAxisGrasps(
            const geometry_msgs::Pose& object_pose,
            grasp_axis_t axis,
            grasp_direction_t direction,
            grasp_rotation_t rotation,
            double hand_roll,
            const GraspData& grasp_data,
            std::vector<moveit_msgs::Grasp>& possible_grasps);
};
}

#endif
