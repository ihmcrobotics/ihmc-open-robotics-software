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
}
