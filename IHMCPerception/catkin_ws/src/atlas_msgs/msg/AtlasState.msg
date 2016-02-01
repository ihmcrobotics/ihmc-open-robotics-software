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
int32 back_lbz  = 0 # v1
int32 back_bkz  = 0 # v3, v4
int32 back_mby  = 1 # v1
int32 back_bky  = 1 # v3, v4
int32 back_ubx  = 2 # v1
int32 back_bkx  = 2 # v3, v4
int32 neck_ay   = 3 # v1
int32 neck_ry   = 3 # v3, v4
int32 l_leg_uhz = 4 # v1
int32 l_leg_hpz = 4 # v3, v4
int32 l_leg_mhx = 5 # v1
int32 l_leg_hpx = 5 # v3, v4
int32 l_leg_lhy = 6 # v1
int32 l_leg_hpy = 6 # v3, v4
int32 l_leg_kny = 7 # v3, v4
int32 l_leg_uay = 8 # v1
int32 l_leg_aky = 8 # v3, v4
int32 l_leg_lax = 9 # v1
int32 l_leg_akx = 9 # v3, v4
int32 r_leg_uhz = 10 # v1
int32 r_leg_hpz = 10 # v3, v4
int32 r_leg_mhx = 11 # v1
int32 r_leg_hpx = 11 # v3, v4
int32 r_leg_lhy = 12 # v1
int32 r_leg_hpy = 12 # v3, v4
int32 r_leg_kny = 13 # v3, v4
int32 r_leg_uay = 14 # v1
int32 r_leg_aky = 14 # v3, v4
int32 r_leg_lax = 15 # v1
int32 r_leg_akx = 15 # v3, v4
int32 l_arm_usy = 16 # v1
int32 l_arm_shy = 16 # v3
int32 l_arm_shz = 16 # v4
int32 l_arm_shx = 17 # v3, v4
int32 l_arm_ely = 18 # v3, v4
int32 l_arm_elx = 19 # v3, v4
int32 l_arm_uwy = 20 # v1
int32 l_arm_wry = 20 # v3, v4
int32 l_arm_mwx = 21 # v1
int32 l_arm_wrx = 21 # v3, v4
int32 r_arm_usy = 22 # v1
int32 r_arm_shy = 22 # v3
int32 r_arm_shz = 22 # v4
int32 r_arm_shx = 23 # v3, v4
int32 r_arm_ely = 24 # v3, v4
int32 r_arm_elx = 25 # v3, v4
int32 r_arm_uwy = 26 # v1
int32 r_arm_wry = 26 # v3, v4
int32 r_arm_mwx = 27 # v1
int32 r_arm_wrx = 27 # v3, v4

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
