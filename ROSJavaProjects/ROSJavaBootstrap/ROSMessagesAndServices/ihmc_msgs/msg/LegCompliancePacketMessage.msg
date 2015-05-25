## LegCompliancePacketMessage
# This packet controls the stiffness of LegJoints. i.e., the maximum force a joint puts outwhen it is (pushed) away from a desired position. This is useful to prevent robot from falling when leg hit things unexpectedly.However, low stiffness (high compliance) can lead to poor joint tracking in the presence of natural joint stiction. Finding a good balance is application specific. In our hybrid velocity+force controller, most force comes from velocity control, therefore, only parameter related to velocity control is exposed.

# maximum allowed force (ratio) from velocity control in the range of [0.0, 1.0]. 1.0 is the maximum stiffness (default) value tuned for fast walking, 0.0 refers to zero velocity control making the jointvery compliant (only force control) but often bad tracking. On Atlas, 0.1-0.3 gives decent tracking for slow motion and yet still compliant.The numbers in the array correspond to joints HPZ, HPX, HPY, KNY, AKY, AKX, respectively.
float32[] max_velocity_deltas

# LEFT or RIGHT leg
# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side


