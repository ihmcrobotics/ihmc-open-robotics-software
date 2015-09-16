package us.ihmc.humanoidRobotics.communication.packets.behaviors;

public enum HumanoidBehaviorType
{
   STOP, SHAKEOUT, DO_NOTHING, SCRIPT, LOCALIZATION, WALK_N_TURN_VALVE, WALK_TO_OBJECT, DEBRIS_TASK,
   WALK_TO_GOAL, DIAGNOSTIC, WHOLE_BODY_IK, TEST, RECEIVE_IMAGE, DRILL_PICK_UP, LOCALIZE_DRILL,PUSH_BUTTON,
   FORCECONTROL_WALL_TASK;

   public static final HumanoidBehaviorType[] values = values();
}
