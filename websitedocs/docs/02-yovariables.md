---
title: Yo Variables
---

To get insight into the controller while it is running some of the internal variables can be visualized from the Simulation Construction Set GUI. This section lists common variables to look at when debugging issues with the controller. It does not list all variables just the ones we commonly use to identify problems with the control algorithms.

Some of the important State Machines:

* `HighLevelState` - usually this will be set on walking. To change it use the `requestedHighLevelState` variable.
* `WalkingState` - this is the main state machine of the walking controller.
* `LeftFootState`, `RightFootState` - the state a foot is in (such as swing or support).

Variables to check the tracking of an end effector (in this case `l_hand`) in task space control mode are listed next. Depending on the type of control done for the end effector some of the variables might not exist or be unused. Some of these variables are vectors or quaternions, meaning that there are multiple variables with name suffixes. To search for variables more efficiently you can use some regex-like expressions. For example, the following is a valid variable search query: `._hand(Desired|Acieved)LinearAcc&(X|Y)$`.

* `l_handDesiredPosition`
* `l_handCurrentPosition`
* `l_handPositionError`
* `l_handDesiredOrientation`
* `l_handCurrentRotation`
* `l_handDesiredRotationVector`
* `l_handCurrentRotationVector`
* `l_handRotationVectorError`
* `l_handDesiredLinearVelocity`
* `l_handLinearVelocityError`
* `l_handDesiredAngularVelocity`
* `l_handAngularVelocityError`
* `l_handDesiredLinearAcceleration` - this is the desired acceleration before the QP
* `l_handAchievedLinearAcceleration` - this is the achieved acceleration by the QP (not the actual robot acceleration)
* `l_handDesiredAngularAcceleration` - this is the desired acceleration before the QP
* `l_handAchievedAngularAcceleration` - this is the achieved acceleration by the QP (not the actual robot acceleration)
* `l_handisPointFBControllerEnabled` - determines the control mode of the end effector
* `l_handisOrientationFBControllerEnabled` - determines the control mode of the end effector
* `l_handisSpatialFBControllerEnabled` - determines the control mode of the end effector

In case you use joint control directly instead of task space control for an end effector you can look at other variables to determine the tracking. The example given uses the `l_arm_elx` joint, but replacing that string with the joint name you are interested with, or even looking at a whole leg using `l_leg_...` is possible. Again, depending on the control mode the specific joint is in these variables might not be created or be unused. In general `qd` denotes a velocity, while `q_d` denotes a desired value `qdd_d`, therefore, is a desired acceleration.

* `q_d_l_arm_elx`
* `q_err_l_arm_elx`
* `qd_d_l_arm_elx`
* `qd_err_l_arm_elx`
* `qdd_d_l_arm_elx` - this is the desired acceleration before the QP
* `qdd_achieved_l_arm_elx` - this is the achieved acceleration by the QP (not the actual robot acceleration)
* `control_enabled_l_arm_elx` - determines whether the joint is in joint control mode