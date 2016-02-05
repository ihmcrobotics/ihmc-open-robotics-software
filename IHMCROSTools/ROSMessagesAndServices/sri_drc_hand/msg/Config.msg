uint8 CONFIG_ACT_NONE=0
uint8 CONFIG_ACT_READ=1
uint8 CONFIG_ACT_WRITE=2
uint8 CONFIG_ACT_WRITE_FLASH=3

# send a set of action/key/value tuples to hw.
uint8[] actions
uint16[] keys
float32[] values
