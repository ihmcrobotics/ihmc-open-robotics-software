package us.ihmc.footstepPlanning;

public enum FootstepPlanningResult
{
   OPTIMAL_SOLUTION,
   SUB_OPTIMAL_SOLUTION,
   TIMED_OUT_BEFORE_SOLUTION,
   NO_PATH_EXISTS,
   SNAPPING_FAILED;

   public static final FootstepPlanningResult[] values = values();

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
}
