# rACT: Action request (activation bit).
# 0x0 - Decativate gripper.
# 0x1 - Activate gripper.

uint8 rACT

# rMOD: Changes the gripper grasping mode.
# 0x0 - Basic mode.
# 0x1 - Pinch mode.
# 0x2 - Wide mode.
# 0x3 - Scissor mode.

uint8 rMOD

# rGTO: "Go To" action moves the gripper fingers to the requested position.
# 0x0 - Stop.
# 0x1 - Go to the requested position.

uint8 rGTO

# rATR: Automatic release routine.
# 0x0 - Stop.
# 0x1 Emergency auto-release.

uint8 rATR

# rICF: Individual Control of Fingers mode. Each finger receives its own command
# unless the gripper is in the Scissor Grasping Mode and the Independent Control
# of Scissor (rICS) is not activated.
# 0x0 - Normal.
# 0x1 - Enable Individual Control of Fingers A, B and C.

uint8 rICF

# rICS: Individual Control of Scissor. The scissor axis moves independenly from
# the grasping mode. When this option is selected, the rMOD bits (grasping mode)
# are ignored as the scissor axis position is defined by the rPRS.
# 0x0 - Normal.
# 0x1 - Enable Individual Control of Scissor. Disable Mode Selection.

uint8 rICS

# rPRA: Target position of the fingers (or finger A only if bit rICF is set).
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 rPRA

# rSPA: Gripper closing or opening speed (or finger A only if bit rICF is set).
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 rSPA

# rFRA: Final grasping force of the gripper (or finger A only if bit rICF is
# set).
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 rFRA

# rPRB: Finger B target position. It is only available if bit rICF is set.
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 rPRB

# rSPB: Finger B speed. It is only available if bit rICF is set.
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 rSPB

# rFRB: Finger B force. It is only available if bit rICF is set.
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 rFRB

# rPRC: Finger C target position. It is only available if bit rICF is set.
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 rPRC

# rSPC: Finger C speed. It is only available if bit rICF is set.
# Setting a speed will not initiate a motion.
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 rSPC

# rFRC: Finger C force. It is only available if bit rICF is set.
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 rFRC

# rPRS: Scissor axis target position. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum position (open).
# 0xFF Maximum position (close).

uint8 rPRS

# rSPS: Scissor axis speed. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum speed.
# 0xFF Maximum speed.

uint8 rSPS

# rFRS: Scissor axis force. It is only applied if the Individual
# Control of Scissor option is selected (bit rICS is set).
# 0x00 Minimum force.
# 0xFF Maximum force.

uint8 rFRS
