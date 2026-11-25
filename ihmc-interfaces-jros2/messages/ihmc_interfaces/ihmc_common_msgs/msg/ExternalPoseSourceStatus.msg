# Status of an external pose source, such as visual-inertial odometry, that sends pelvis poses to the state estimator.

# The source's current session. A new session starts each time the source resets its world frame.
# It matches the frame_id of the StampedPosePackets the source sends.
string session_id

# Wall time, in nanoseconds, at which the current session started. Poses timestamped earlier belong to a previous session.
int64 session_start_time

# What the source is doing, for display. For example: waiting for start, restarting, running.
string state

# Poses sent and rejected by the source in the current session.
int64 accepted_count
int64 rejected_count

# Why the source last rejected a pose or is waiting, for display.
string detail
