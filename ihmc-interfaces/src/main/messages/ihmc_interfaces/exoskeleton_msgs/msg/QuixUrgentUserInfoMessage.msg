# This message is part of the Quix controller API.
# This message is used to notify the crutch display of some important system state that might otherwise be hard to recognize.

byte EMPTY = 0
byte ESTOP_PRESSED = 1
byte MOTOR_OVER_TEMP = 2
byte LOW_BATTERY = 3

uint32 sequence_id


byte urgent_user_info_name