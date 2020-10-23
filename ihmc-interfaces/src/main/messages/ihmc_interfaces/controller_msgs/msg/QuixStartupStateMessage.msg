byte INITIALIZING = 0
byte WAIT_FOR_USER = 1
byte RECOVERABLE_FAULT = 2
byte FATAL_FAULT = 3
byte LIMP = 4
byte WIGGLE = 5
byte HOLD_ALL_READY = 6
byte HOLD_SOME_LOCKED = 7
byte CALL_BEHAVIOR = 8

uint32 sequence_id

byte startup_state_name 255