package us.ihmc.behaviors.sequence.actions;

public enum WalkActionExecutionState
{
   TRIGGERED,
   FOOTSTEP_PLANNING,
   PLANNING_FAILED,
   PLANNING_SUCCEEDED,
   PLAN_COMMANDED,
   PLAN_EXECUTION_COMPLETE;

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
