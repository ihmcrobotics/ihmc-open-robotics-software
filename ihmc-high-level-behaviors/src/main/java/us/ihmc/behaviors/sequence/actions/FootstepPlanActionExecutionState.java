package us.ihmc.behaviors.sequence.actions;

public enum FootstepPlanActionExecutionState
{
   FOOTSTEP_PLANNING,
   PLANNING_FAILED,
   PLANNING_SUCCEEDED,
   PLAN_COMMANDED;

   public static final FootstepPlanActionExecutionState[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlanActionExecutionState fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
