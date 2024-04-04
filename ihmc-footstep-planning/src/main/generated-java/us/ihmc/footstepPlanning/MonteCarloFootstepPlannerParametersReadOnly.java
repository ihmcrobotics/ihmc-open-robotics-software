package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface MonteCarloFootstepPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getReset()
   {
      return get(reset);
   }

   default int getNumberOfIterations()
   {
      return get(numberOfIterations);
   }

   default int getNumberOfSimulations()
   {
      return get(numberOfSimulations);
   }

   default double getTimeoutDuration()
   {
      return get(timeoutDuration);
   }

   default double getFeasibleContactReward()
   {
      return get(feasibleContactReward);
   }

   default double getGoalReward()
   {
      return get(goalReward);
   }

   default double getSuccessReward()
   {
      return get(successReward);
   }

   default int getMaxNumberOfVisitedNodes()
   {
      return get(maxNumberOfVisitedNodes);
   }

   default int getMaxNumberOfChildNodes()
   {
      return get(maxNumberOfChildNodes);
   }

   default double getMaxTransferHeight()
   {
      return get(maxTransferHeight);
   }

   default double getMaxTransferDepth()
   {
      return get(maxTransferDepth);
   }

   default double getMaxTransferYaw()
   {
      return get(maxTransferYaw);
   }

   default double getExplorationConstant()
   {
      return get(explorationConstant);
   }

   default int getInitialValueCutoff()
   {
      return get(initialValueCutoff);
   }

   default int getMaxTreeDepth()
   {
      return get(maxTreeDepth);
   }

   default double getSearchInnerRadius()
   {
      return get(searchInnerRadius);
   }

   default double getSearchOuterRadius()
   {
      return get(searchOuterRadius);
   }

   default int getSearchSkipSize()
   {
      return get(searchSkipSize);
   }

   default double getMaximumStepWidth()
   {
      return get(maximumStepWidth);
   }

   default double getMaximumStepLength()
   {
      return get(maximumStepLength);
   }

   default double getMinimumStepWidth()
   {
      return get(minimumStepWidth);
   }

   default double getMinimumStepLength()
   {
      return get(minimumStepLength);
   }

   default double getDecayFactor()
   {
      return get(decayFactor);
   }

   default double getMinimumContactValue()
   {
      return get(minimumContactValue);
   }

   default double getMaximumContactValue()
   {
      return get(maximumContactValue);
   }

   default double getYawDelta()
   {
      return get(yawDelta);
   }
}
