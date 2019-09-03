package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

public enum PawStepPlanningResult
{
   OPTIMAL_SOLUTION,
   SUB_OPTIMAL_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   SNAPPING_FAILED,
   PLANNER_FAILED;

   public static final PawStepPlanningResult[] values = values();

   public static PawStepPlanningResult generateRandomResult(Random random)
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

   public static PawStepPlanningResult getWorstResult(PawStepPlanningResult resultA, PawStepPlanningResult resultB)
   {
      byte aResult = (byte) MathTools.clamp(resultA.toByte(), OPTIMAL_SOLUTION.toByte(), PLANNER_FAILED.toByte());
      byte bResult = (byte) MathTools.clamp(resultB.toByte(), OPTIMAL_SOLUTION.toByte(), PLANNER_FAILED.toByte());
      return fromByte((byte) Math.max(aResult, bResult));
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static PawStepPlanningResult fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
