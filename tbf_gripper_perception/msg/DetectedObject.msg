# message typ is designed to collaborate with the planning scene from moveit
# especially - moveit_msgs/CollisionObject.msg object
Header header
string id

# shape
shape_msgs/SolidPrimitive primitive
geometry_msgs/Pose primitive_pose

# mesh
shape_msgs/Mesh mesh
geometry_msgs/Pose mesh_pose

# class name and classification probability
string class
float32  class_p