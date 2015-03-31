#Structure for desired foot step data

int32 step_index
int32 foot_index # Foot index (Left/Right); see AtlasFootId enum. 
float64 duration # Step duration
geometry_msgs/Vector3 position # Foot position, in Atlas world frame
float64 yaw # Foot orientation yaw component, in Atlas world frame
geometry_msgs/Vector3 normal # Foot ground normal, in Atlas world frame
float64 swing_height # Step apex swing height as measured from the midpoint between the feet. 
