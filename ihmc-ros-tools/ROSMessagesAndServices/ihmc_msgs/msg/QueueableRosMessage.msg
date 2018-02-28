## QueueableRosMessage
# 

# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the
# controller will start directly at the first trajectory point. Otherwise the controller will prepend
# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must
# carry the ID of the message it should be queued to.  - The very first message of a list of queued
# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last
# trajectory point time of the previous message.  - The controller will queue the joint trajectory
# messages as a per joint basis. The first trajectory point has to be greater than zero.
int8 execution_mode

# Only needed when using QUEUE mode, it refers to the message Id to which this message should be
# queued to. It is used by the controller to ensure that no message has been lost on the way. If a
# message appears to be missing (previousMessageId different from the last message ID received by the
# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the
# ID of the last received message.
int64 previous_message_id

# The time to delay this message on the controller side before being executed.
float64 execution_delay_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


