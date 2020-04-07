package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum FootstepPlanningResult
{
   PLANNING,
   FOUND_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   INVALID_GOAL,
   MAXIMUM_ITERATIONS_REACHED,
   EXCEPTION,
   HALTED;

   public static final FootstepPlanningResult[] values = values();

   public static FootstepPlanningResult generateRandomResult(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public boolean validForExecution()
   {
      switch (this)
      {
      case FOUND_SOLUTION:
      case PLANNING:
         return true;
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

   public static FootstepPlanningResult fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      else
         return values[enumAsByte];
   }
}
