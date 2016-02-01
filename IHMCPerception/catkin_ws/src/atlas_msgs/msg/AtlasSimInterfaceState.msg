# For interfacing Boston Dynamics' AtlasSimInterface Dynamics Behavior Library
# Feedback from AtlasSimInterface Controller after calling process_control_input
# This ROS message should track AtlasControlOutput struct in
# AtlasSimInterfaceTypes.h.
# With the exception of addition of k_effort to provide user a way to switch
# to/from PID servo control in AtlasPlugin.cpp on a per joint basis.

int32 NO_ERRORS                        =  0    # no error detected
int32 ERROR_UNSPECIFIED                = -1    # unspecified error
int32 ERROR_VALUE_OUT_OF_RANGE         = -2    # passed value is out of range
int32 ERROR_INVALID_INDEX              = -3    # passed index is invalid (too low or too high)
int32 ERROR_FAILED_TO_START_BEHAVIOR   = -4    # robot failed to start desired behavior
int32 ERROR_NO_ACTIVE_BEHAVIOR         = -5    # robot has no active behavior
int32 ERROR_NO_SUCH_BEHAVIOR           = -6    # behavior doesn't exist
int32 ERROR_BEHAVIOR_NOT_IMPLEMENTED   = -7    # behavior exists but not implemented
int32 ERROR_TIME_RAN_BACKWARD          = -8    # a time earlier than previous times was given

Header header

int32 error_code                         # error code returned by
                                         # process_control_input.
                                         # See AtlasSimInterfaceTypes.h
                                         # AtlasErrorCode for list of enums.
                                         # The list is mimic'd here above.

int32 current_behavior                   # current active behavior.
int32 desired_behavior                   # desired behavior specified by usesr
                                         # input. This may lag from
                                         # current_behavior by a few simulation
                                         # steps.

# below are information from AtlasControlOutput in AtlasSimInterfaceTypes.h

float64[30] f_out                        # torque command from BDI controller. The first 28 values are used by Atlas versions < 4

atlas_msgs/AtlasPositionData pos_est     # Position and velocity estimate of robot pelvis

geometry_msgs/Pose[2] foot_pos_est      # World position estimate for feet
                                         # 0 - left, 1 - right

atlas_msgs/AtlasBehaviorFeedback behavior_feedback
atlas_msgs/AtlasBehaviorStepFeedback step_feedback
atlas_msgs/AtlasBehaviorStandFeedback stand_feedback
atlas_msgs/AtlasBehaviorWalkFeedback walk_feedback
atlas_msgs/AtlasBehaviorManipulateFeedback manipulate_feedback

# additional vector for transitioning from servo model in AtlasPlugin
# to BDI servo.

uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.

