package us.ihmc.footstepPlanning;

public interface FootstepPlanReadOnly
{
   int getNumberOfSteps();

   PlannedFootstepReadOnly getFootstep(int footstepIndex);

   double getFinalTransferSplitFraction();

   double getFinalTransferWeightDistribution();

   default boolean isEmpty()
   {
      return getNumberOfSteps() == 0;
   }
}
