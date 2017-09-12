# Command to move the HANDLE hand
# 

# The different control types
uint8 VELOCITY = 1
uint8 POSITION = 2
uint8 CURRENT  = 3
uint8 VOLTAGE  = 4
uint8 ANGLE    = 5

# The control type for each motor.
int32[5] type

# The value to set.
int32[5] value

# Whether or not to control each motor.
bool[5] valid

# To be added if/when this becomes a service call:
#---
#bool ok
#string reason
