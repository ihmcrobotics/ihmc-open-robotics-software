# This message contains the state of one joint of the pr2 robot.
# This message is specificly designed for the pr2 robot. 
# A generic joint state message can be found in sensor_msgs::JointState

# the name of the joint
string name

# the time at which these joint statistics were measured
time timestamp

# the position of the joint in radians
float64 position

# the velocity of the joint in radians per second
float64 velocity

# the measured joint effort 
float64 measured_effort

# the effort that was commanded to the joint.
# the actual applied effort might be different
# because the safety code can limit the effort
# a joint can apply
float64 commanded_effort

# a flag indicating if the joint is calibrated or not
bool is_calibrated

# a flag inidcating if the joint violated one of its position/velocity/effort limits
# in the last publish cycle
bool violated_limits

# the total distance travelled by the joint, measured in radians.
float64 odometer

# the lowest position reached by the joint in the last publish cycle
float64 min_position

# the highest position reached by the joint in the last publish cycle
float64 max_position

# the maximum absolute velocity reached by the joint in the last publish cycle
float64 max_abs_velocity

# the maximum absolute effort applied by the joint in the last publish cycle
float64 max_abs_effort
