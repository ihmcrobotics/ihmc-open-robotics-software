int8 WAKE_UP=0

int8 REINITIALIZE=1

int8 SLEEP=2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value 255
int8 requested_toolbox_state

# If true and the toolbox supports logging, all messages to and from the toolbox are logged.
# Can only be requested for WAKE_UP and REINITIALIZE. SLEEP will automatically end the toolbox log.
bool request_logging


