package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

public enum PawPlannerStatus
{
   IDLE,
   PLANNING_PATH,
   PLANNING_STEPS;

   public static final PawPlannerStatus[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawPlannerStatus fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
