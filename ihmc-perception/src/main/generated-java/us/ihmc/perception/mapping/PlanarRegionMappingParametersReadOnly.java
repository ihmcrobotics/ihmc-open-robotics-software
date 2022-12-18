package us.ihmc.perception.mapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.mapping.PlanarRegionMappingParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PlanarRegionMappingParametersReadOnly extends StoredPropertySetReadOnly
{
   default double getUpdateAlphaTowardsMatch()
   {
      return get(updateAlphaTowardsMatch);
   }

   default int getAngleThresholdBetweenNormals()
   {
      return get(angleThresholdBetweenNormals);
   }

   default double getOrthogonalDistanceThreshold()
   {
      return get(orthogonalDistanceThreshold);
   }

   default double getMaxInterRegionDistance()
   {
      return get(maxInterRegionDistance);
   }
}
