# multi_step walking trajectory parameters
uint32 step_index              # Step index, matlab style, starting from 1,
                               # monotonically increasing during walking
                               #  resets to 1 if robot leaves walk behaviors
uint32 foot_index              # Foot_index can be LEFT_FOOT or RIGHT_FOOT
float64 duration               # Step duration, when in doubt, 0.63s is a
                               # good guess.
geometry_msgs/Pose pose        # Foot pose in Atlas world frame
float64 swing_height           # Step apex swing height measured form the
                               # midpoint between the feet.
