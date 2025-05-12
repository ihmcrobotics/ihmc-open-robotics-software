package us.ihmc.perception.filters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilteringParametersBasics extends DepthImageFilteringParametersReadOnly, StoredPropertySetBasics
{
   default void setNeighborhoodSize(int neighborhoodSize)
   {
      set(DepthImageFilteringParameters.neighborhoodSize, neighborhoodSize);
   }

   default void setCosineThreshold(double cosineThreshold)
   {
      set(DepthImageFilteringParameters.cosineThreshold, cosineThreshold);
   }

   default void setNormalThreshold(double normalThreshold)
   {
      set(DepthImageFilteringParameters.normalThreshold, normalThreshold);
   }
}
