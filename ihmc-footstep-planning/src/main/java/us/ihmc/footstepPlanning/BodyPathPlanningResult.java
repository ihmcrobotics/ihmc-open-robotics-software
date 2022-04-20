package us.ihmc.footstepPlanning;

public enum BodyPathPlanningResult
{
   PLANNING,
   FOUND_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   INVALID_GOAL,
   MAXIMUM_ITERATIONS_REACHED,
   EXCEPTION,
   HALTED;

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

   public boolean terminalResult()
   {
      return this != PLANNING;
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
