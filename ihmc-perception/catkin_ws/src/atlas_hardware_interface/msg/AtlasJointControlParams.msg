# Structure for specifying controller parameters.
# This structure contains the gains to be applied to a joint.
# q, qd, f are sensed position, velocity, torque, from AtlasJointState
# q_d, qd_d, f_d are desired position, velocity, torque, from AtlasJointDesired
# The final joint command will be:
#
#k_q_p * ( q_d - q ) + k_q_i * 1/s * ( q_d - q ) + k_qd_p * ( qd_d - qd ) + k_f_p * ( f_d - f ) + ff_qd * qd + ff_qd_d * qd_d + ff_f_d * f_d + ff_const
# Gains for controllers run on the field computer side are prefixed with 'ex'

float64 k_q_p                  # Position error gain, in N*m/rad.
float64 ex_k_q_out_p           # Position error gain on output side encoders, in N*m/rad.
float64 k_q_i                  # Integral of position error gain, in N*m/(rad*s).
float64 ex_k_q_out_i           # Integral of position error gain on output side encoders, in N*m/(rad*s).
float64 ex_k_q_out_i_clamp     # Max. abs. value to which the integeral position error is clamped.
float64 ex_k_q_out_i_liveband  # Integral active when within this band
float64 k_qd_p                 # Derivative error gain, in N*m/(rad/s).
float64 k_f_p                  # Proportional force feedback gain.
float64 ff_qd                  # Feedforward velocity gain.
float64 ff_qd_d                # Feedforward desired velocity gain.
float64 ex_ff_qd_d             # Feedforward desired velocity gain.
float64 ff_f_d                 # Feedforward desired force gain.
float64 ex_velforce            # Toggle for pre-mapped velocity/force feedforward term.
float64 ff_const               # Constant force term.
float64 qerr_int_abs_max       # Max. abs. value to which the integeral position error is clamped.

