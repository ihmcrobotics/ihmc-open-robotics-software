# This message is part of the IHMC whole-body controller API.
# General message carrying the information needed to safely queue messages.

byte EXECUTION_MODE_OVERRIDE = 0
byte EXECUTION_MODE_QUEUE = 1
byte EXECUTION_MODE_STREAM = 2

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When EXECUTION_MODE_OVERRIDE is chosen:
# - For trajectory messages: the time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point.
#   Otherwise the controller will prepend a first trajectory point at the current desired position.
# When EXECUTION_MODE_QUEUE is chosen:
# - The message must carry the ID of the message it should be queued to.
# - The very first message of a list of queued messages has to be an EXECUTION_MODE_OVERRIDE message.
# - For trajectory messages:
#   - the trajectory point times are relative to the the last trajectory point time of the previous message.
#   - the time of the first trajectory point has to be greater than zero.
#   - When joint-space trajectory: the controller will queue the joint trajectory messages as a per joint basis.
# When EXECUTION_MODE_STREAM is chosen:
# The controller is expecting a continuous stream of messages sent at high frequency.
# A trajectory message that is part of a stream is expected to have a single trajectory point with a time set to 0.
# A stream can be terminated with either an override message or a stream message with a desired velocity of 0.
# While it is possible to stream messages using the EXECUTION_MODE_OVERRIDE, discontinuities in velocity will be experienced.
# These discontinuities can be solved by using EXECUTION_MODE_STREAM with stream_integration_duration greater than the time interval between two consecutive messages of a stream.
byte execution_mode 0
# Defines a unique ID for this message. Only needed when you want to queue another message to this message.
int64 message_id -1
# Only needed when using EXECUTION_MODE_QUEUE mode, it refers to the message_id to which this message should be queued to.
# It is used by the controller to ensure that no message has been lost on the way.
# If a message appears to be missing (previous_message_id different from the last message_id received by the controller), the motion is aborted.
int64 previous_message_id 0
# The time to delay this message on the controller side before being executed.
float64 execution_delay_time
# When receiving a trajectory message that is part of a stream, the controller will extrapolate the trajectory point in the future using a simple first order integration over the given duration.
# This integration allows to improve continuity of execution for streams.
# If no new message is received once the integration duration has elapsed, the controller will hold the desired position and reset the desired velocity to 0.
float64 stream_integration_duration 0