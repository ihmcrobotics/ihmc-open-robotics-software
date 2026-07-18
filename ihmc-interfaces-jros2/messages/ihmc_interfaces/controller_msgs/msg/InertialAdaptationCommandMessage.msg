# Per-hand operator "grasping" state, streamed from the VR teleoperation interface to the onboard
# controller to drive the online inertial estimator's per-limb adaptation gate. While a hand is
# grasping (its joystick trigger is held, e.g. as the operator manually loads that forearm), the
# estimator raises the process noise on that forearm so its inertial estimate can be driven; a
# non-grasping hand's forearm is held pinned at its nominal inertia so a one-sided load cannot be
# mis-attributed onto it. Published at the teleop stream rate; the controller treats a stale stream
# as "signal lost" and freezes a still-grasping forearm rather than letting it relax.

bool left_hand_grasping

bool right_hand_grasping
