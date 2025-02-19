
# True if this is a request to servo the robot
# This is used to enable and ramp up the robot's controller gains, to go into freeze or stand prep
bool servo_robot

# True is this is a request to immediately zero the gains which can be violent and dangerous
# This will also turn off the HPU
bool unservo_immediately

# True if this is a request to slowly ramp down the robot's controller gains to zero, in prepration for robot shutdown
# This will also turn off the HPU at the end
bool unservo_slowly


