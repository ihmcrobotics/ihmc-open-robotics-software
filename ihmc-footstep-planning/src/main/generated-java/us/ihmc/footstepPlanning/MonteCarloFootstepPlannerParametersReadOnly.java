package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface MonteCarloFootstepPlannerParametersReadOnly extends StoredPropertySetReadOnly
{
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

   default double getMaxTransferDistance()
   {
      return get(maxTransferDistance);
   }

   default double getMinTransferDistance()
   {
      return get(minTransferDistance);
   }

   default double getMaxTransferYaw()
   {
      return get(maxTransferYaw);
   }

   default int getGoalMargin()
   {
      return get(goalMargin);
   }

   default double getFeasibleContactCutoff()
   {
      return get(feasibleContactCutoff);
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
}
