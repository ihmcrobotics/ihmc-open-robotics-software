package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.activeMapping.StancePoseParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface StancePoseParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getContactCostWeight()
   {
      return get(contactCostWeight);
   }

   default double getMaxContactValue()
   {
      return get(maxContactValue);
   }

   default double getNominalStanceDistance()
   {
      return get(nominalStanceDistance);
   }

   default int getSearchWindowSize()
   {
      return get(searchWindowSize);
   }

   default double getSearchWindowResolution()
   {
      return get(searchWindowResolution);
   }
}
