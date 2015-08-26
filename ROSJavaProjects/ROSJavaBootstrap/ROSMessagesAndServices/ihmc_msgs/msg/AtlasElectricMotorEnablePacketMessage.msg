## AtlasElectricMotorEnablePacketMessage
# Specifies a specific electric motor in the Atlas forearm to power on or off.

# Options for motorEnableEnum
uint8 L_ARM_WRY=0 # left upper wrist pitch
uint8 L_ARM_WRX=1 # left wrist roll
uint8 L_ARM_WRY2=2 # left lower wrist pitch
uint8 R_ARM_WRY=3 # right upper wrist pitch
uint8 R_ARM_WRX=4 # right wrist roll
uint8 R_ARM_WRY2=5 # right lower wrist pitch
uint8 motor_enable_enum

bool enable

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id


