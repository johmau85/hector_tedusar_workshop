string object_name    # name of object in planning scene

string object_type    # type of object which should be grasped(eg: zylinder, quader, cube...)

geometry_msgs/PoseStamped object_pose # pose of object (position represents midpoint of object)
geometry_msgs/Vector3 object_grasping_direction # optional: good grasping direction of object (e.g. normal to wall etc.)
  


