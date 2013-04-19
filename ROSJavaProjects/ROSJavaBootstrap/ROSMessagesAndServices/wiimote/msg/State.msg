#
#          Wiimote State message containing one complete Wiimote state

# Note: For ease of manipulation this message often uses
#       int8[] when a bit vector would work. One might
#       wish to use uint8[], but then Python takes
#       the respective structure as a string and disallows
#       item assignment.

int8    INVALID       = -1
float32 INVALID_FLOAT = -1.0

int8 MSG_BTN_1     = 0
int8 MSG_BTN_2     = 1
int8 MSG_BTN_A     = 2
int8 MSG_BTN_B     = 3
int8 MSG_BTN_PLUS  = 4
int8 MSG_BTN_MINUS = 5
int8 MSG_BTN_LEFT  = 6
int8 MSG_BTN_RIGHT = 7
int8 MSG_BTN_UP    = 8
int8 MSG_BTN_DOWN  = 9
int8 MSG_BTN_HOME  = 10
int8 MSG_BTN_Z     = 0
int8 MSG_BTN_C     = 1
int8 MSG_CLASSIC_BTN_X       = 0
int8 MSG_CLASSIC_BTN_Y       = 1
int8 MSG_CLASSIC_BTN_A       = 2
int8 MSG_CLASSIC_BTN_B       = 3
int8 MSG_CLASSIC_BTN_PLUS    = 4
int8 MSG_CLASSIC_BTN_MINUS   = 5
int8 MSG_CLASSIC_BTN_LEFT    = 6
int8 MSG_CLASSIC_BTN_RIGHT   = 7
int8 MSG_CLASSIC_BTN_UP      = 8
int8 MSG_CLASSIC_BTN_DOWN    = 9
int8 MSG_CLASSIC_BTN_HOME    = 10
int8 MSG_CLASSIC_BTN_L       = 11
int8 MSG_CLASSIC_BTN_R       = 12
int8 MSG_CLASSIC_BTN_ZL      = 13
int8 MSG_CLASSIC_BTN_ZR      = 14

#-----------------------------
# Header
#----------------------

Header header

#----------------------
# Gyro reading
#-----------------------------
# In radians/sec. If reading is invalid,
# for instance b/c no gyro is attached to the Wii, then
# set first element of covariance to -1 (same as imu_data.msg):
# Covariance matrix (roll, pitch, yaw) in radians^2/sec^2.

geometry_msgs/Vector3 angular_velocity_zeroed
geometry_msgs/Vector3 angular_velocity_raw
float64[9] angular_velocity_covariance

#----------------------
# Accelerometer reading
#-----------------------------
# Acceleration in m/sec^2. Covariance matrix
# (x,y,z) in m^2/sec^4.  (all same as imu_data.msg)

geometry_msgs/Vector3 linear_acceleration_zeroed
geometry_msgs/Vector3 linear_acceleration_raw
float64[9] linear_acceleration_covariance

#------------------------------
# Nunchuk Accelerometer reading
#------------------------------
geometry_msgs/Vector3 nunchuk_acceleration_zeroed
geometry_msgs/Vector3 nunchuk_acceleration_raw

#-----------------
# Nunchuk Joystick
#-----------------
float32[2] nunchuk_joystick_zeroed
float32[2] nunchuk_joystick_raw

#----------------------
# Wiimote buttons
#-----------------------------
# Mappings from Wiimote button name
# to array position are defined above.
#
bool[11] buttons
bool[2] nunchuk_buttons

#----------------------
# Wiimote LED states:
#-----------------------------

bool[4] LEDs

#----------------------
# Wiimote Rumble
#-----------------------------
# State (True or False)

bool rumble

#----------------------
# IR Light sensor (Camera)
#-----------------------------
# The Wiimote handles up to four light sources, 
# and the wiimote_node.py software is written to 
# that limit as well. For future expansion
# we make the following array extensible, rather
# than locking its length down to four:

wiimote/IrSourceInfo[] ir_tracking

#----------------------
# Wiimote battery
#-----------------------------
# A battery reading consists of two numbers: 
# the battery percentage, and the raw reading.
# Maximum battery is 208 units (unknown how this
# relates to electrical properties): 

float32 raw_battery
float32 percent_battery

#----------------------
# Time of most recent zeroing:
#-----------------------------

time zeroing_time

#----------------------
# Error vector
#-----------------------------
# For error condition definitions see wiimoteConstants.py
# Value of zero means all is well. (Currently NOT used).

uint64 errors
