# Joint Command Message
# This structure contains the gains to be applied to a joint.
# The controller is a PID with feedforward desired torque:
#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
#   k_effort * (
#     kp_position     * ( position - measured_position )       +
#     ki_position     * 1/s * ( position - measured_position ) +
#     kd_position     * s * ( position - measured_position ) +
#     kp_velocity     * ( velocity - measured_velocity )     +
#     effort ) +
#   (1 - k_effort)  * effort_bdi
#
#  Where effort_bdi comes from AtlasSimInterface Dynamics Behavior Library.
#
Header header

float64[] position
float64[] velocity
float64[] effort

float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity
uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.

float32[] i_effort_min
float32[] i_effort_max

uint8 desired_controller_period_ms # max allowed controller update period in milli-seconds simulation time for controller synchronization.  See SynchronizationStatistics.msg for tracking synchronization status.
