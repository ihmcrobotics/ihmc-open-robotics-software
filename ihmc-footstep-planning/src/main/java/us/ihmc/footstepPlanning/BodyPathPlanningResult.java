package us.ihmc.footstepPlanning;

public enum BodyPathPlanningResult
{
   FOUND_SOLUTION,
   NO_PATH_EXISTS,
   EXCEPTION;

   public static final BodyPathPlanningResult[] values = values();

   public boolean validForExecution()
   {
      switch(this)
      {
         case FOUND_SOLUTION:
            return true;
         case EXCEPTION:
         default:
            return false;
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static BodyPathPlanningResult fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      else
         return values[enumAsByte];
   }
}
