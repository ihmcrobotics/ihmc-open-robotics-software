package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum HumanoidBehaviorType
{
   STOP, TEST, WALK_TO_LOCATION, WALK_TO_GOAL, DIAGNOSTIC, PICK_UP_BALL,TURN_VALVE,EXAMPLE_BEHAVIOR, BALL_DETECTION, TEST_PIPELINE, TEST_STATEMACHINE;

   public static final HumanoidBehaviorType[] values = values();
}
