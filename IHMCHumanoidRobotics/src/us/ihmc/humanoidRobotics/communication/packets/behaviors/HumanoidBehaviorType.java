package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum HumanoidBehaviorType
{
   STOP, TEST, WALK_TO_LOCATION, WALK_TO_GOAL, DIAGNOSTIC, PICK_UP_BALL,RESET_ROBOT,TURN_VALVE,WALK_THROUGH_DOOR,
   EXAMPLE_BEHAVIOR, BALL_DETECTION, TEST_PIPELINE, TEST_STATEMACHINE,
   FOLLOW_FIDUCIAL_50, LOCATE_FIDUCIAL, WAlK_OVER_TERRAIN, DEBUG_PARTIAL_FOOTHOLDS;

   public static final HumanoidBehaviorType[] values = values();
}
