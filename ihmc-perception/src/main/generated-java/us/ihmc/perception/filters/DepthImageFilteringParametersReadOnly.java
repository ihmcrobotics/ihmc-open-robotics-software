package us.ihmc.perception.filters;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.filters.DepthImageFilteringParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilteringParametersReadOnly extends StoredPropertySetReadOnly
{
   default int getNeighborhoodSize()
   {
      return get(neighborhoodSize);
   }

   default double getCosineThreshold()
   {
      return get(cosineThreshold);
   }

   default double getNormalThreshold()
   {
      return get(normalThreshold);
   }
}
