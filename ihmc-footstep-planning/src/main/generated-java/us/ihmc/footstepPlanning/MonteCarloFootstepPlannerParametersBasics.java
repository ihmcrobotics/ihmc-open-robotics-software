package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface MonteCarloFootstepPlannerParametersBasics extends MonteCarloFootstepPlannerParametersReadOnly, StoredPropertySetBasics
{
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
}
