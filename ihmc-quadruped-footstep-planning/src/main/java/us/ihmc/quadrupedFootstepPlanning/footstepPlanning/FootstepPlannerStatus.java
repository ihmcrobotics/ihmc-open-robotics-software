package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

public enum FootstepPlannerStatus
{
   IDLE,
   PLANNING_PATH,
   PLANNING_STEPS;

   public static final FootstepPlannerStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
