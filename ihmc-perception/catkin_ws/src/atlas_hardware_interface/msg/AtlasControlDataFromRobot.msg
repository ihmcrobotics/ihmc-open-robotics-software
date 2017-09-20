# Structure for current robot state to be streamed from the robot.

Header header
int64 processed_to_robot_packet_seq_id # Sequence id of last to-robot packet received and processed.
int64 control_data_version_mismatch # Whether version of control data matches with on-robot software.
atlas_hardware_interface/AtlasRobotRunState run_state # State of robot.
uint64 robot_status_flags # Set of AtlasRobotStatus flags.
float64 pump_inlet_pressure # Pressure at pump inlet, in PSI.
float64 pump_supply_pressure # Pressure at pump supply, in PSI.
float64 pump_return_pressure # Pump return pressure, in PSI.
float64 air_sump_pressure # Air pressure in accusump.
float64 pump_time_meter # How many hours pump has been run since installation
float64 current_pump_rpm # Current RPM of pump
float64 pump_supply_temperature # Temperature of oil at pump supply, in degC; avoid > 90 deg; shut down at 95
float64 motor_temperature # Temperature of motor, in degC; avoid > 125 deg; shut down at 150
float64 motor_driver_temperature # Temperature of motor, in degC; avoid > 55 deg; shut down at 60
atlas_hardware_interface/AtlasJointState[28] j # Current joint states
atlas_hardware_interface/AtlasActuatorState[28] act # Pressure readings from hydraulic actuators
atlas_hardware_interface/AtlasExtendedJointFeedback[28] jfeed # User filtered joint states
sensor_msgs/Imu filtered_imu # Packet of filtered IMU data.
atlas_hardware_interface/AtlasRawIMUData[15] raw_imu # Last n packets unfiltered IMU data.
geometry_msgs/Wrench[2] foot_sensors # State of foot sensors.
float64[8] foot_strain_gauges # State of strain gauges, in volts
geometry_msgs/Wrench[2] wrist_sensors # State of wrist sensors.
float64[12] wrist_strain_gauges # State of strain gauges, in volts
atlas_hardware_interface/AtlasPositionData pos_est # World position estimate for reference when giving desired step positions (not updated except in behavior modes where noted)
geometry_msgs/Vector3[2] foot_pos_est # World position estimate for feet (not updated except in behavior modes where noted)
atlas_hardware_interface/AtlasRobotBehavior current_behavior # Current behavior of robot.
atlas_hardware_interface/AtlasBehaviorFeedback behavior_feedback # General behavior feedback.
atlas_hardware_interface/AtlasBehaviorStandFeedback stand_feedback # Feedback specific to the Stand behavior.
atlas_hardware_interface/AtlasBehaviorStepFeedback step_feedback # Feedback specific to the Step behavior.
atlas_hardware_interface/AtlasBehaviorWalkFeedback walk_feedback # Feedback specific to the Walk behavior.
atlas_hardware_interface/AtlasBehaviorManipulateFeedback manipulate_feedback # Feedback specific to the Manipulate behavior.
uint64 sensor_head_pps_timestamp # Timestamp of sensor head PPS signal.

