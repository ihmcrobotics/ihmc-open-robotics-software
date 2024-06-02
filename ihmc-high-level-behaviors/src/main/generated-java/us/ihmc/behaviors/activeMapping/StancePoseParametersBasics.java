package us.ihmc.behaviors.activeMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface StancePoseParametersBasics extends StancePoseParametersReadOnly, StoredPropertySetBasics
{
   default void setContactCostWeight(double contactCostWeight)
   {
      set(StancePoseParameters.contactCostWeight, contactCostWeight);
   }

   default void setMaxContactValue(double maxContactValue)
   {
      set(StancePoseParameters.maxContactValue, maxContactValue);
   }

   default void setNominalStanceDistance(double nominalStanceDistance)
   {
      set(StancePoseParameters.nominalStanceDistance, nominalStanceDistance);
   }

   default void setSearchWindowSize(int searchWindowSize)
   {
      set(StancePoseParameters.searchWindowSize, searchWindowSize);
   }

   default void setSearchWindowResolution(double searchWindowResolution)
   {
      set(StancePoseParameters.searchWindowResolution, searchWindowResolution);
   }
}
