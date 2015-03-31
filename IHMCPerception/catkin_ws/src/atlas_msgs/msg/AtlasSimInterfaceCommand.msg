# For interfacing AtlasSimInterface Dynamics Behavior Library
# This ROS message should track behavior commands in AtlasControlInput struct
# inside AtlasSimInterfaceTypes.h.
# With the exception of addition of k_effort to provide user a way to switch
# to/from PID servo control in AtlasPlugin.cpp on a per joint basis.

Header header

# permissible values for behavior
# Updated to match the values in AtlasRobotInterface/AtlasInterfaceTypes.h.
int32 NONE              =  0 # Robot currently not in any behavior; software freeze, controls disabled.
int32 FREEZE            =  1 # safety mode
int32 STAND_PREP        =  2 # stand-prep (AtlasSimInterface documentation)
int32 STAND             =  3 # stand
int32 WALK              =  4 # multi-step walk
int32 STEP              =  5 # single step walk
int32 MANIPULATE        =  6 # stand and allows manipulation.
int32 USER              =  7 # disable AtlasSimInterface updates, rely on
                             # /atlas/atlas_command or /atlas/joint_commands

int32 behavior                # can be one of
                              # NONE, USER, FREEZE, STAND_PREP
                              # WALK, STEP, STAND, MANIPULATE

# multi_step walking trajectory parameters
atlas_msgs/AtlasBehaviorWalkParams walk_params

# parameters for single_step behavior
atlas_msgs/AtlasBehaviorStepParams step_params

# parameters for standing behavior
atlas_msgs/AtlasBehaviorStandParams stand_params

# parameters for stand and manipulate
atlas_msgs/AtlasBehaviorManipulateParams manipulate_params

# additional vector for transitioning from servo model in AtlasPlugin
# to BDI servo.

uint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, 
                       # at run time, a double between 0 and 1 is obtained
                       # by dividing by 255.0d.

# servo control in AtlasSimInterface
float64[] position
float64[] velocity
float64[] effort

float32[] kp_position
float32[] ki_position
float32[] kp_velocity
