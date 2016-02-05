# A definition of a collision operation
# E.g. ("gripper",COLLISION_SET_ALL,ENABLE) will enable collisions 
# between the gripper and all objects in the collision space

string object1
string object2
string COLLISION_SET_ALL="all"
string COLLISION_SET_OBJECTS="objects"
string COLLISION_SET_ATTACHED_OBJECTS="attached"

# The penetration distance to which collisions are allowed. This is 0.0 by default.
float64 penetration_distance

# Flag that determines whether collisions will be enabled or disabled for the pair of objects specified above
int32 operation
int32 DISABLE=0
int32 ENABLE=1
