#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
# testing everything a robot needs
Header header

# Default joint indices used when publishing the
# JointCommands joint_states topic below
# For exmaple, if you subscribe to this message, then
# msg.joint_states.position[atlas_msgs::AtlasStates::back_lbz] gives back
# the position of the back_lbz.
int32 back_bkz  = 0
int32 back_bky  = 1
int32 back_bkx  = 2
int32 neck_ry   = 3
int32 l_leg_hpz = 4
int32 l_leg_hpx = 5
int32 l_leg_hpy = 6
int32 l_leg_kny = 7
int32 l_leg_aky = 8
int32 l_leg_akx = 9
int32 r_leg_hpz = 10
int32 r_leg_hpx = 11
int32 r_leg_hpy = 12
int32 r_leg_kny = 13
int32 r_leg_aky = 14
int32 r_leg_akx = 15
int32 l_arm_shz = 16
int32 l_arm_shx = 17
int32 l_arm_ely = 18
int32 l_arm_elx = 19
int32 l_arm_wry = 20
int32 l_arm_wrx = 21
int32 r_arm_shz = 22
int32 r_arm_shx = 23
int32 r_arm_ely = 24
int32 r_arm_elx = 25
int32 r_arm_wry = 26
int32 r_arm_wrx = 27

# repeating data from osrf_msgs/JointCommands as joint_states
float32[] position
float32[] velocity
float32[] effort
float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity
float32[] i_effort_min
float32[] i_effort_max

uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.


#sensor_msgs/Imu imu 
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 linear_acceleration

#atlas_msgs/ForceTorqueSensors force_torque_sensors
geometry_msgs/Wrench l_foot
geometry_msgs/Wrench r_foot
geometry_msgs/Wrench l_hand
geometry_msgs/Wrench r_hand
