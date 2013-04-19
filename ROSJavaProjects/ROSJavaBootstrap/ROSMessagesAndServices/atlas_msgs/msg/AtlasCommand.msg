# Joint Command Message
# This structure contains the gains to be applied to a joint.
# The controller is a PID with feedforward desired torque:
#
# This message has been carefully constructed to be less
# than 1500 in size when serialized, to accommodate transfer
# UDP.
#
#   kp_position     * ( position - measured_position )       +
#   ki_position     * 1/s * ( position - measured_position ) +
#   kd_position     * s * ( position - measured_position ) +
#   kp_velocity    * ( velocity - measured_velocity )     +
#   effort
#
Header header

float64[] position
float64[] velocity
float64[] effort

float32[] kp_position
float32[] ki_position
float32[] kd_position
float32[] kp_velocity

float32[] i_effort_min
float32[] i_effort_max
