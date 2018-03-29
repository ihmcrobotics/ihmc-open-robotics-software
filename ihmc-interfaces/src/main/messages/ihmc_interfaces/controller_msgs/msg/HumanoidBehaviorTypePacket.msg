# This message is part of the IHMC humanoid behavior module.

byte STOP = 0
byte TEST = 1
byte WALK_TO_LOCATION = 2
byte WALK_TO_GOAL = 3
byte DIAGNOSTIC = 4
byte PICK_UP_BALL = 5
byte RESET_ROBOT = 6
byte TURN_VALVE = 7
byte WALK_THROUGH_DOOR = 8
byte EXAMPLE_BEHAVIOR = 9
byte BALL_DETECTION = 10
byte TEST_PIPELINE = 11
byte TEST_STATEMACHINE = 12
byte FOLLOW_FIDUCIAL_50 = 13
byte LOCATE_FIDUCIAL = 14
byte WALK_OVER_TERRAIN = 15
byte FOLLOW_VALVE = 16
byte LOCATE_VALVE = 17
byte WALK_OVER_TERRAIN_TO_VALVE = 18
byte DEBUG_PARTIAL_FOOTHOLDS = 19
byte WALK_TO_GOAL_ANYTIME_PLANNER = 20
byte TEST_ICP_OPTIMIZATION = 21
byte TEST_GC_GENERATION = 22
byte TEST_SMOOTH_ICP_PLANNER = 23
byte PUSH_AND_WALK = 24
byte COLLABORATIVE_TASK = 25
byte FIRE_FIGHTING = 26
byte CUTTING_WALL = 27
byte REPEATEDLY_WALK_FOOTSTEP_LIST = 28

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

byte humanoid_behavior_type 255