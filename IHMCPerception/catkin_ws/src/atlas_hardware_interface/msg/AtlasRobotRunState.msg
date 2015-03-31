# Enumerations identifying run states of the robot.
# AtlasRobotRunState
# The robot will be in only one run state at any one time. It can only enter states under certain conditions.

uint8 state

#Enum for the possible run states
uint8 RUN_STATE_IDLE = 0
    # Initial state.
    # Entries:
    #     from RUN_STATE_STOP
    #     from RUN_STATE_START
    # Exits:
    #     to RUN_STATE_START when function start() called
uint8 RUN_STATE_START = 1
    # Entries:
    #     from RUN_STATE_IDLE
    # Exits:
    #     to RUN_STATE_CONTROL automatically if pump start completes or pump not used
    #     to RUN_STATE_STOP automatically if pump start fails
uint8 RUN_STATE_CONTROL = 3
    # Entries:
    #     from RUN_STATE_START
    # Exits:
    #     to RUN_STATE_STOP if stop() called
    #     to RUN_STATE_STOP on critical fault
uint8 RUN_STATE_STOP = 5
    # Entries:
    #     from RUN_STATE_CONTROL
    #     from critical fault
    # Exits:
    #     to RUN_STATE_IDLE automatically

