package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface MonteCarloFootstepPlannerParametersBasics extends MonteCarloFootstepPlannerParametersReadOnly, StoredPropertySetBasics
{
   default void setReset(boolean reset)
   {
      set(MonteCarloFootstepPlannerParameters.reset, reset);
   }

   default void setNumberOfIterations(int numberOfIterations)
   {
      set(MonteCarloFootstepPlannerParameters.numberOfIterations, numberOfIterations);
   }

   default void setNumberOfSimulations(int numberOfSimulations)
   {
      set(MonteCarloFootstepPlannerParameters.numberOfSimulations, numberOfSimulations);
   }

   default void setTimeoutDuration(double timeoutDuration)
   {
      set(MonteCarloFootstepPlannerParameters.timeoutDuration, timeoutDuration);
   }

   default void setFeasibleContactReward(double feasibleContactReward)
   {
      set(MonteCarloFootstepPlannerParameters.feasibleContactReward, feasibleContactReward);
   }

   default void setGoalReward(double goalReward)
   {
      set(MonteCarloFootstepPlannerParameters.goalReward, goalReward);
   }

   default void setMaxNumberOfVisitedNodes(int maxNumberOfVisitedNodes)
   {
      set(MonteCarloFootstepPlannerParameters.maxNumberOfVisitedNodes, maxNumberOfVisitedNodes);
   }

   default void setMaxNumberOfChildNodes(int maxNumberOfChildNodes)
   {
      set(MonteCarloFootstepPlannerParameters.maxNumberOfChildNodes, maxNumberOfChildNodes);
   }

   default void setMaxTransferHeight(double maxTransferHeight)
   {
      set(MonteCarloFootstepPlannerParameters.maxTransferHeight, maxTransferHeight);
   }

   default void setMaxTransferDepth(double maxTransferDepth)
   {
      set(MonteCarloFootstepPlannerParameters.maxTransferDepth, maxTransferDepth);
   }

   default void setMaxTransferYaw(double maxTransferYaw)
   {
      set(MonteCarloFootstepPlannerParameters.maxTransferYaw, maxTransferYaw);
   }

   default void setFeasibleContactCutoff(double feasibleContactCutoff)
   {
      set(MonteCarloFootstepPlannerParameters.feasibleContactCutoff, feasibleContactCutoff);
   }

   default void setExplorationConstant(double explorationConstant)
   {
      set(MonteCarloFootstepPlannerParameters.explorationConstant, explorationConstant);
   }

   default void setInitialValueCutoff(int initialValueCutoff)
   {
      set(MonteCarloFootstepPlannerParameters.initialValueCutoff, initialValueCutoff);
   }

   default void setMaxTreeDepth(int maxTreeDepth)
   {
      set(MonteCarloFootstepPlannerParameters.maxTreeDepth, maxTreeDepth);
   }

   default void setSidedYawOffset(double sidedYawOffset)
   {
      set(MonteCarloFootstepPlannerParameters.sidedYawOffset, sidedYawOffset);
   }

   default void setSearchYawBand(double searchYawBand)
   {
      set(MonteCarloFootstepPlannerParameters.searchYawBand, searchYawBand);
   }

   default void setSearchInnerRadius(double searchInnerRadius)
   {
      set(MonteCarloFootstepPlannerParameters.searchInnerRadius, searchInnerRadius);
   }

   default void setSearchOuterRadius(double searchOuterRadius)
   {
      set(MonteCarloFootstepPlannerParameters.searchOuterRadius, searchOuterRadius);
   }

   default void setSearchSkipSize(int searchSkipSize)
   {
      set(MonteCarloFootstepPlannerParameters.searchSkipSize, searchSkipSize);
   }

   default void setTransferYawCost(double transferYawCost)
   {
      set(MonteCarloFootstepPlannerParameters.transferYawCost, transferYawCost);
   }

   default void setTransferHeightCost(int transferHeightCost)
   {
      set(MonteCarloFootstepPlannerParameters.transferHeightCost, transferHeightCost);
   }

   default void setTransferDistanceCost(double transferDistanceCost)
   {
      set(MonteCarloFootstepPlannerParameters.transferDistanceCost, transferDistanceCost);
   }
}
