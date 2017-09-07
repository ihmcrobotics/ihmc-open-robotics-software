# This is basic collision message
# it is used in HandleCollisions to build an array

string frame_id
# finger[0]/proximal_link
# finger[0]/distal_link
# finger[1]/proximal_link
# finger[1]/distal_link
# finger[2]/proximal_link
# finger[2]/distal_link
# base_link

int32 sensor_id
# index of sensor

float32 intensity

# location of sensor on the surface of the finger in the link frame
float32 x
float32 y
float32 z
