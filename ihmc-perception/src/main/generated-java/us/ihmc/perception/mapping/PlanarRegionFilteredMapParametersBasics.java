package us.ihmc.perception.mapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PlanarRegionFilteredMapParametersBasics extends PlanarRegionFilteredMapParametersReadOnly, StoredPropertySetBasics
{
   default void setUpdateAlphaTowardsMatch(double updateAlphaTowardsMatch)
   {
      set(PlanarRegionFilteredMapParameters.updateAlphaTowardsMatch, updateAlphaTowardsMatch);
   }

   default void setAngleThresholdBetweenNormals(int angleThresholdBetweenNormals)
   {
      set(PlanarRegionFilteredMapParameters.angleThresholdBetweenNormals, angleThresholdBetweenNormals);
   }

   default void setOrthogonalDistanceThreshold(double orthogonalDistanceThreshold)
   {
      set(PlanarRegionFilteredMapParameters.orthogonalDistanceThreshold, orthogonalDistanceThreshold);
   }

   default void setMaxInterRegionDistance(double maxInterRegionDistance)
   {
      set(PlanarRegionFilteredMapParameters.maxInterRegionDistance, maxInterRegionDistance);
   }
}
