package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

public enum PawStepPlannerTargetType
{
   POSE_BETWEEN_FEET, FOOTSTEPS;

   public static PawStepPlannerTargetType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawStepPlannerTargetType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
