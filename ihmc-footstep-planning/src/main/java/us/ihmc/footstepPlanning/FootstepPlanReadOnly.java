package us.ihmc.footstepPlanning;

public interface FootstepPlanReadOnly
{
   int getNumberOfSteps();

   PlannedFootstepReadOnly getFootstep(int footstepIndex);

   default boolean isEmpty()
   {
      return getNumberOfSteps() == 0;
   }
}
