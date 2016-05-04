## AtlasElectricMotorEnablePacketRosMessage
# Specifies a specific electric motor in the Atlas forearm to power on or off.

# The Enum value of the motor to enable
uint8 motor_enable_enum

# Boolean for enable state; true for enable, false for disable.
bool enable

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "atlas_electric_motor_packet_enum" enum values:
uint8 L_ARM_WRY=0 # left wrist roll
uint8 L_ARM_WRX=1 # left upper wrist pitch
uint8 L_ARM_WRY2=2 # left lower wrist pitch
uint8 R_ARM_WRY=3 # right wrist roll
uint8 R_ARM_WRX=4 # right upper wrist pitch
uint8 R_ARM_WRY2=5 # right lower wrist pitch

