#This captures acceleration measurements from the 3-dof accelerometer in the hand of the PR2
#Units are meters / second / second
#Vectors should be <X, Y, Z> in the frame of the gripper

#The communication with the accelerometer is at approximately 3khz, but there is only good timestamping every 1ms
#This means the samples should be interpreted as all having come from the 1 ms before the time in the header

Header header
geometry_msgs/Vector3[] samples
