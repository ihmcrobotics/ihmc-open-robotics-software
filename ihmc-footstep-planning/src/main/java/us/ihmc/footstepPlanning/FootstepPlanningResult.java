package us.ihmc.footstepPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum FootstepPlanningResult
{
   OPTIMAL_SOLUTION,
   SUB_OPTIMAL_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   SNAPPING_FAILED,
   PLANNER_FAILED,
   INVALID_GOAL;

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

   public static FootstepPlanningResult getWorstResult(FootstepPlanningResult resultA, FootstepPlanningResult resultB)
   {
      byte aResult = (byte) MathTools.clamp(resultA.toByte(), OPTIMAL_SOLUTION.toByte(), INVALID_GOAL.toByte());
      byte bResult = (byte) MathTools.clamp(resultB.toByte(), OPTIMAL_SOLUTION.toByte(), INVALID_GOAL.toByte());
      return fromByte((byte) Math.max(aResult, bResult));
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
