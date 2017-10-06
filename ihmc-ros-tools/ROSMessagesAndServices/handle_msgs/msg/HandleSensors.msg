# All of the sensors on the HANDLE hand.
#
# NOTE: In general, the order of the arrays is: [F1, F2, F3, F3 Ant., Spread].
# Where: F1 is analogous to your right hand index finger, F2 is analogous to 
# your right middle finger, F3 is analogous to your right thumb, F3 Ant. is 
# the antagonistic motor for F3, and Spread is the motor which controls the 
# rotation of F1 and F2.
#

# Currently only used for time stamp.  Time stamp set from Overo's clock.
# Not guaranteed to be in sync with the current time.
Header header

# The hall effect sensor on the finger motors.  
# 24 counts per motor revolution x 300 motor revolutions for one full spindle
# rotation.  
# 3500 to put finger at approx. 90 degrees
# 6000 to close finger gently
# 7000 to close finger tightly
#
# [F1, F2, F3, F3 Ant.]
int32[4] motorHallEncoder

# The temperature of the finger motor windings, in Celsius.
#
# [F1, F2, F3, F3 Ant.]
float32[4] motorWindingTemp

# The air temperature as measured inside the housing, in Celsius.
float32 airTemp

# The motor velocity in RPM.  (Hall encoder ticks per minute)
#
# [F1, F2, F3, F3 Ant.]
int32[4] motorVelocity

# The temperature of the motor housing, in Celsius.
# Note that the housing temp for the Spread motor is not populated, so will
# not have a logical value.
#
# [F1, F2, F3, F3 Ant., Spread]
float32[5] motorHousingTemp

# The motor current in amps.
#
# [F1, F2, F3, F3 Ant., Spread]
float32[5] motorCurrent

# The tactile array for each finger.  In units of ADC counts.
# Note there are 12 proximal and 10 distal sensors.
#
# [F1, F2, F3]
Finger[3] fingerTactile

# The tactile temperature array for each finger, in Celsius.
# Note there are 12 proximal and 10 distal sensors.
#
# [F1, F2, F3]
Finger[3] fingerTactileTemp

# The tactile array for the palm.  In units of ADC counts.
float32[48] palmTactile

# The tactile temperature array for the palm, in Celsius.
float32[48] palmTactileTemp

# The encoder on the F1 / F2 rotation. 8.533 ticks per degree.
# 3072 ticks per 180 degree rotation. (not possible).
# 768 ticks to rotate the fingers 90 degrees for a "T" grasp.
# 512 ticks to rotate the fingers 60 degrees for a spherical grasp.
int32 fingerSpread

# The proximal joint angle.  Approx 2.84 ticks per degree.
# 1024 ticks per full revolution. (not possible)
# 256 ticks to put finger at approx. 90 degrees
# 435 ticks to close finger gently
# 445 ticks to close finger tightly
#
# [F1, F2, F3]
int32[3] proximalJointAngle

# The finger distal joint flexture angle
# [F1, F2, F3]
# Note there are 4 readings on either side of the joint.
Finger[3] distalJointAngle

# The accelerometer in the proximal link of each finger.  In G's.
# This sensor is not populated at this time.
#
# [F1, F2, F3]
geometry_msgs/Vector3[3] proximalAcceleration

# The accelerometer in the distal link of each finger.  In G's.
# Z axis points out the back of the finger.
# Y axis points out the finger tip.
# X axis points to the left when looking at the finger pad.
#
# [F1, F2, F3]
geometry_msgs/Vector3[3] distalAcceleration

# In this sensor message, which devices actually contributed data.
# The order of the array is:
#  0: Palm (Traffic Cop)
#  1: Finger 1 Proximal
#  2: Finger 1 Distal
#  3: Finger 2 Proximal
#  4: Finger 2 Distal
#  5: Finger 3 Proximal
#  6: Finger 3 Distal
#  7: Motor for Finger 1
#  8: Motor for Finger 2
#  9: Motor for Finger 3
# 10: Motor for Finger 3 Antagonist
# 11: Tactile Palm
bool[12] responses

# How many of the the last 100 messages had data from each device.
# This gives a good indication of communication quality between the devices.
# Note that this only counts sensor message history.  If you have a motor that
# is in thermal shutdown and non-responsive, the device may look fine here.
# However there will be errors in the motorError array below.
# The order of the array is:
#  0: Palm (Traffic Cop)
#  1: Finger 1 Proximal
#  2: Finger 1 Distal
#  3: Finger 2 Proximal
#  4: Finger 2 Distal
#  5: Finger 3 Proximal
#  6: Finger 3 Distal
#  7: Motor for Finger 1
#  8: Motor for Finger 2
#  9: Motor for Finger 3
# 10: Motor for Finger 3 Antagonist
# 11: Tactile Palm
int8[12] responseHistory

# The last received error from each motor.
# The order of the motors in the array:
# [F1, F2, F3, F3 Ant., Spread]
# The error codes are:
#   0 = No Error
#   1 = Traffic Cop checksum error
#   2 = Unknown Command
#   3 = Bad Address
#   4 = Traffic Cop timeout waiting for response
#  17 = Overo checksum error
#  18 = Overo data length error
#  20 = Overo timout waiting for response
# 128 = Thermal Protection Delay
# 129 = Mandatory Cool Down 
# 130 = Request out of range
int16[5] motorError
