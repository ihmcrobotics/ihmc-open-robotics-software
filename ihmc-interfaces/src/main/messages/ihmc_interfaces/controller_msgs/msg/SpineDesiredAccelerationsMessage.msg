# This message is part of the IHMC whole-body controller API.
# This message gives the user the option to bypass IHMC feedback controllers for the spine joints by sending desired spine joint accelerations.
# One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# The desired joint acceleration information.
DesiredAccelerationsMessage desired_accelerations