## AtlasElectricMotorEnablePacketMessage
# Specifies a specific electric motor in the Atlas forearm to power on or off.

# Options for motorEnableEnum
# L_ARM_WRY = 0 - left upper wrist pitch
# L_ARM_WRX = 1 - left wrist roll
# L_ARM_WRY2 = 2 - left lower wrist pitch
# R_ARM_WRY = 3 - right upper wrist pitch
# R_ARM_WRX = 4 - right wrist roll
# R_ARM_WRY2 = 5 - right lower wrist pitch
uint8 motor_enable_enum

bool enable


