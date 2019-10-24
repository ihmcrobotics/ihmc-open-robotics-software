package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

public enum PawStepPlannerStatus
{
   IDLE,
   PLANNING_PATH,
   PLANNING_STEPS;

   public static final PawStepPlannerStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawStepPlannerStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
