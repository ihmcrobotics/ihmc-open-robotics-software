package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

public interface FootstepPlannerParameters
{
   double getMaximumStepReach();

   double getMaximumStepWidth();

   double getMaximumStepCycleDistance();

   double getMinimumStepLength();

   double getMinimumStepWidth();
}
