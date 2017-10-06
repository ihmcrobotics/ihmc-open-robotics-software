#Structure for control output data to be streamed to robot sim.
# This structure is for holding data that is to be streamed to the robot at a high rate. Most of this data will be parameters for actuator control.

Header header
atlas_hardware_interface/AtlasJointState[28] j # Setpoints for robot joints.
atlas_hardware_interface/AtlasJointControlParams[28] jparams # Joint gains, etc. for robot joints.
uint32[2] power_hand # Whether to apply power to specified hand; specify HAND_LEFT or HAND_RIGHT.
atlas_hardware_interface/AtlasHandMass[2] hand_mass # Mass data for each hand; specify HAND_LEFT or HAND_RIGHT.
uint32 full_ub_cntrl # Whether to control the full upper body or just the neck during BDI behaviors.
uint32 comms_failure_disable_pump # Disable hydraulic power if no packets have been received for a specified time (typ. 250ms). Otherwise, stop behaviors and/or freeze.
atlas_hardware_interface/AtlasBehaviorStandParams stand_params # Input parameters for Stand behavior.
atlas_hardware_interface/AtlasBehaviorStepParams step_params # Input parameters for Step behavior.
atlas_hardware_interface/AtlasBehaviorWalkParams walk_params # Input parameters for Walk behavior.
atlas_hardware_interface/AtlasBehaviorManipulateParams manipulate_params # Input parameters for Manipulate behavior.

