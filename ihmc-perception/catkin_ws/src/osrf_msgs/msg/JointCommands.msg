# Joint Command Message
# This structure contains the gains to be applied to a joint.
# The controller is a PID with feedforward desired torque:
#
#   kp_position     * ( position - measured_position )       +
#   ki_position     * 1/s * ( position - measured_position ) +
#   kd_position     * s * ( position - measured_position ) +
#   kp_velocity    * ( velocity - measured_velocity )     +
#   effort
#
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

float64[] kp_position
float64[] ki_position
float64[] kd_position
float64[] kp_velocity

float64[] i_effort_min
float64[] i_effort_max
