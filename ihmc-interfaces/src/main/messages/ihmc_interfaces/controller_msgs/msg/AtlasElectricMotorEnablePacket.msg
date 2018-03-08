# Specifies a specific electric motor in the Atlas forearm to power on or off.

byte L_ARM_WRY = 0
byte L_ARM_WRX = 1
byte L_ARM_WRY2 = 2
byte R_ARM_WRY = 3
byte R_ARM_WRX = 4
byte R_ARM_WRY2 = 5

# The Enum value of the motor to enable
byte atlas_electric_motor_packet_enum_enable 255
# Boolean for enable state; true for enable, false for disable.
bool enable