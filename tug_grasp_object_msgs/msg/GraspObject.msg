 # name of object in planning scene
string object_name   

# type of object which should be grasped(eg: zylinder, quader, cube...)
string object_type    

# pose of object (position represents midpoint of object)
geometry_msgs/PoseStamped object_pose 

# optional: good grasping direction of object (e.g. normal to wall etc.)
geometry_msgs/Vector3 object_grasping_direction 
  


