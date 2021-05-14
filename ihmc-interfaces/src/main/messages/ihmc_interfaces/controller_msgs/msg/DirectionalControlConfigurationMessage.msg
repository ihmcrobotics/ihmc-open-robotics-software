# This message contains the control behavior for Directional Control.

uint32 sequence_id

# Set to true to cause the robot to walk. Setting this to false will cause the robot to stop walking. 
# However, the robot will still finish the current step.
bool enable_walking false

# Specify the profile name to use. Profiles contain definitions of walking and stepping
# parameters that modify how aggressive or conservative steps will be.
# Assuming it exists, directional control will be initialized with the "default" profile.
# If this field is left empty, no alteration will be made to the profile.
# By default, profiles are stored in the directory ~/.ihmc/joystick_step_app and are
# in .ini format. 
# For more details, see UserProfileManager and JoystickStepParameters.  
string profile_name ""