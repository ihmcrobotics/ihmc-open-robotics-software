Header header

atlas_hardware_interface/AtlasRobotRunState run_state # State of robot.

uint64 robot_status_flags # Set of AtlasRobotStatus flags.

atlas_hardware_interface/AtlasRobotBehavior current_behavior # Current behavior of robot.

atlas_hardware_interface/AtlasBehaviorFeedback behavior_feedback # General behavior feedback.
atlas_hardware_interface/AtlasBehaviorStandFeedback stand_feedback # Feedback specific to the Stand behavior.
atlas_hardware_interface/AtlasBehaviorStepFeedback step_feedback # Feedback specific to the Step behavior.
atlas_hardware_interface/AtlasBehaviorWalkFeedback walk_feedback # Feedback specific to the Walk behavior.
atlas_hardware_interface/AtlasBehaviorManipulateFeedback manipulate_feedback # Feedback specific to the Manipulate behavior.
    
