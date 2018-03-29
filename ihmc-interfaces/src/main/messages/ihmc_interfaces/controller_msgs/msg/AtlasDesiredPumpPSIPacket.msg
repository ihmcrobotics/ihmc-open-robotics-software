# Send a request to change the desired PSI of the Atlas hydraulic pump.

# As of March 2018, the header for this message is only use for its sequence ID.
std_msgs/Header header

# The desired hydraulic pump PSI.
int32 desired_pump_psi