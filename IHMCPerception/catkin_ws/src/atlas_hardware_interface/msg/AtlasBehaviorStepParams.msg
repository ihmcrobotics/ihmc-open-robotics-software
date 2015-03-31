# Structure for parameters for the Step behavior.
# This structure contains parameters needed for controlling the Step behavior. Refer to the Atlas Software and Control Manual for details on when and how parameters should be set.

atlas_hardware_interface/AtlasBehaviorStepData desired_step # Desired position of next step. 
atlas_hardware_interface/AtlasBehaviorStepSpec desired_step_spec # Desired position of next step. 
int32 use_spec
int32 use_relative_step_height     # Whether to use relative step height. 
int32 use_demo_walk                # Whether to use demo walk for step data. 

