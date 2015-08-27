##LastReceivedMessage
# Feedback from the last command received by the controller

# Message type of the last command message received by the controller
string type

# id of the last command message received by the controller receive
int64 unique_id

# timestamp in nanoseconds the controller received the command message with unique_id
int64 receive_timestamp

# time in nanoseconds since the command was received. The current time equals time_since_last_received + receive_timestamp
int64 time_since_last_received
