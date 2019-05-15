package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

public enum FootstepPlannerTargetType
{
   POSE_BETWEEN_FEET, FOOTSTEPS;

   public static FootstepPlannerTargetType[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerTargetType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}
