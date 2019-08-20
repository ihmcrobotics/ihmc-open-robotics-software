package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

public enum PawPlannerTargetType
{
   POSE_BETWEEN_FEET, FOOTSTEPS;

   public static PawPlannerTargetType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawPlannerTargetType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
