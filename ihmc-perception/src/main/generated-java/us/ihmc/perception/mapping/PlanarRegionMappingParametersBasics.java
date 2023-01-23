package us.ihmc.perception.mapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PlanarRegionMappingParametersBasics extends PlanarRegionMappingParametersReadOnly, StoredPropertySetBasics
{
   default void setUpdateAlphaTowardsMatch(double updateAlphaTowardsMatch)
   {
      set(PlanarRegionMappingParameters.updateAlphaTowardsMatch, updateAlphaTowardsMatch);
   }

   default void setSimilarityThresholdBetweenNormals(double similarityThresholdBetweenNormals)
   {
      set(PlanarRegionMappingParameters.similarityThresholdBetweenNormals, similarityThresholdBetweenNormals);
   }

   default void setOrthogonalDistanceThreshold(double orthogonalDistanceThreshold)
   {
      set(PlanarRegionMappingParameters.orthogonalDistanceThreshold, orthogonalDistanceThreshold);
   }

   default void setMaxInterRegionDistance(double maxInterRegionDistance)
   {
      set(PlanarRegionMappingParameters.maxInterRegionDistance, maxInterRegionDistance);
   }

   default void setMinimumOverlapThreshold(double minimumOverlapThreshold)
   {
      set(PlanarRegionMappingParameters.minimumOverlapThreshold, minimumOverlapThreshold);
   }
}
