package us.ihmc.footstepPlanning;

public interface FootstepPlanReadOnly
{
   int getNumberOfSteps();

   PlannedFootstepReadOnly getFootstep(int footstepIndex);
}
