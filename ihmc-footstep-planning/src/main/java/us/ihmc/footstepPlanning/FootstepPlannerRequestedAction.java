package us.ihmc.footstepPlanning;

public enum FootstepPlannerRequestedAction
{
   HALT,
   PUBLISH_PARAMETERS;

   private static final FootstepPlannerRequestedAction[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlannerRequestedAction fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }


}
