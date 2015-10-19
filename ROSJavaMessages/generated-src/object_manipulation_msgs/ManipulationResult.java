package object_manipulation_msgs;

public interface ManipulationResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ManipulationResult";
  static final java.lang.String _DEFINITION = "# Result codes for manipulation tasks\n\n# task completed as expected\n# generally means you can proceed as planned\nint32 SUCCESS = 1\n\n# task not possible (e.g. out of reach or obstacles in the way)\n# generally means that the world was not disturbed, so you can try another task\nint32 UNFEASIBLE = -1\n\n# task was thought possible, but failed due to unexpected events during execution\n# it is likely that the world was disturbed, so you are encouraged to refresh\n# your sensed world model before proceeding to another task\nint32 FAILED = -2\n\n# a lower level error prevented task completion (e.g. joint controller not responding)\n# generally requires human attention\nint32 ERROR = -3\n\n# means that at some point during execution we ended up in a state that the collision-aware\n# arm navigation module will not move out of. The world was likely not disturbed, but you \n# probably need a new collision map to move the arm out of the stuck position\nint32 ARM_MOVEMENT_PREVENTED = -4\n\n# specific to grasp actions\n# the object was grasped successfully, but the lift attempt could not achieve the minimum lift distance requested\n# it is likely that the collision environment will see collisions between the hand/object and the support surface\nint32 LIFT_FAILED = -5\n\n# specific to place actions\n# the object was placed successfully, but the retreat attempt could not achieve the minimum retreat distance requested\n# it is likely that the collision environment will see collisions between the hand and the object\nint32 RETREAT_FAILED = -6\n\n# indicates that somewhere along the line a human said \"wait, stop, this is bad, go back and do something else\"\nint32 CANCELLED = -7\n\n# the actual value of this error code\nint32 value\n";
  static final int SUCCESS = 1;
  static final int UNFEASIBLE = -1;
  static final int FAILED = -2;
  static final int ERROR = -3;
  static final int ARM_MOVEMENT_PREVENTED = -4;
  static final int LIFT_FAILED = -5;
  static final int RETREAT_FAILED = -6;
  static final int CANCELLED = -7;
  int getValue();
  void setValue(int value);
}
