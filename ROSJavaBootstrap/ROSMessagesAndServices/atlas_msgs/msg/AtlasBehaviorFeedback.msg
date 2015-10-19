# mirrors AtlasBehaviorFeedback
#
# Transition flags:
#    - STATUS_TRANSITION_IN_PROGRESS
#
#        A transition is in progress.
#
#    - STATUS_TRANSITION_SUCCESS
#
#        Successful transition.
#
#    - STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR
#
#        Failed to transition; unknown behavior.
#
#    - STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR
#
#        Denied request for an illegal behavior transition.  This may
#        happen if a transition to a new behavior is requested without
#        going through a required intermediate behavior. (e.g., can't
#        go from Walk straight to Manipulate.)
#
#    - STATUS_FAILED_TRANS_COM_POS
#
#        Failed to transition; the position of the COM is too far from
#        the center of support.
#
#    - STATUS_FAILED_TRANS_COM_VEL
#
#        Failed to transition; the COM velocity too high.
#
#    - STATUS_FAILED_TRANS_VEL
#
#        Failed to transition; some joint velocities too high.
#
#  \em Warnings:
#
#    - STATUS_WARNING_AUTO_TRANS
#
#        An automatic transition occurred; see behavior specific
#        feedback for reason.
#
#  \em Errors:
#
#    - STATUS_ERROR_FALLING
#
#        COM below acceptable threshold, cannot recover.

# copied from AtlasBehaviorFlags
uint32 STATUS_OK                            = 0
uint32 STATUS_TRANSITION_IN_PROGRESS        = 1
uint32 STATUS_TRANSITION_SUCCESS            = 2
uint32 STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR = 4
uint32 STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR = 8
uint32 STATUS_FAILED_TRANS_COM_POS          = 16
uint32 STATUS_FAILED_TRANS_COM_VEL          = 32
uint32 STATUS_FAILED_TRANS_VEL              = 64
uint32 STATUS_WARNING_AUTO_TRANS            = 128
uint32 STATUS_ERROR_FALLING                 = 256

uint32 status_flags  # can be one of above

int32 trans_from_behavior_index  # use this as a parm to get_behavior_at_index() to get behavior string
int32 trans_to_behavior_index  # use this as a parm to get_behavior_at_index() to get behavior string
