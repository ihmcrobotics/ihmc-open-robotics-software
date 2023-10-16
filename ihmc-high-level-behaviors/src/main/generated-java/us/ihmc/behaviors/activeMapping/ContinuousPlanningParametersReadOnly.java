package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.activeMapping.ContinuousPlanningParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface ContinuousPlanningParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getActiveMapping()
   {
      return get(activeMapping);
   }

   default boolean getPauseContinuousWalking()
   {
      return get(pauseContinuousWalking);
   }
}
