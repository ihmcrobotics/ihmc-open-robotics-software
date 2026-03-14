package us.ihmc.behaviors.behaviorTree.action.actions;

public enum WalkActionExecutionState
{
   FOOTSTEP_PLANNING,
   PLANNING_FAILED,
   PLANNING_SUCCEEDED,
   PLAN_COMMANDED;

   public static final WalkActionExecutionState[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static WalkActionExecutionState fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
