package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousPlanningParametersBasics extends ContinuousPlanningParametersReadOnly, StoredPropertySetBasics
{
   default void setActiveMapping(boolean activeMapping)
   {
      set(ContinuousPlanningParameters.activeMapping, activeMapping);
   }

   default void setPauseContinuousWalking(boolean pauseContinuousWalking)
   {
      set(ContinuousPlanningParameters.pauseContinuousWalking, pauseContinuousWalking);
   }
}
