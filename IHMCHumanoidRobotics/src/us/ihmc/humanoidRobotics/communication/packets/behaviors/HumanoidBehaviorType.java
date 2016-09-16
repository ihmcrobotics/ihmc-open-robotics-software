package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum HumanoidBehaviorType
{
   STOP, TEST, WALK_TO_LOCATION, WALK_TO_GOAL, DIAGNOSTIC, PICK_UP_BALL, BALL_DETECTION;

   public static final HumanoidBehaviorType[] values = values();
}
