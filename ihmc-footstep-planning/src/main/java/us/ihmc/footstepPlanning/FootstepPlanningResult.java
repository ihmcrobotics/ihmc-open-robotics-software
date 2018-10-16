package us.ihmc.footstepPlanning;

import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum FootstepPlanningResult
{
   OPTIMAL_SOLUTION,
   SUB_OPTIMAL_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   SNAPPING_FAILED,
   PLANNER_FAILED;

   public static final FootstepPlanningResult[] values = values();

   public static FootstepPlanningResult generateRandomResult(Random random)
   {
      return values[RandomNumbers.nextInt(random, 0, values.length - 1)];
   }

   public boolean validForExecution()
   {
      switch (this)
      {
      case OPTIMAL_SOLUTION:
      case SUB_OPTIMAL_SOLUTION:
         return true;
      default:
         return false;
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static FootstepPlanningResult fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
