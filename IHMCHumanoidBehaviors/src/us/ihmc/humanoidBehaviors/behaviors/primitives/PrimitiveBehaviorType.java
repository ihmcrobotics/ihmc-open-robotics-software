package us.ihmc.humanoidBehaviors.behaviors.primitives;

public enum PrimitiveBehaviorType
{
   IDLE,
   FOOTSTEP_LIST,
   HAND_TRAJECTORY,
   END_EFFECTOR_LOAD_BEARING,
   HEAD_ORIENTATION,
   COM_HEIGHT,
   FOOT_POSE,
   PELVIS_POSE,
   CHEST_TRAJECTORY,
   HIGH_LEVEL_STATE,
   FINGER_STATE,
   LOOK_AT_BEHAVIOR;

   public static final PrimitiveBehaviorType[] values = values();
}
