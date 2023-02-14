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

   default void setMinimumPlanarRegionArea(double minimumPlanarRegionArea)
   {
      set(PlanarRegionMappingParameters.minimumPlanarRegionArea, minimumPlanarRegionArea);
   }

   default void setMinimumBoundingBoxSize(double minimumBoundingBoxSize)
   {
      set(PlanarRegionMappingParameters.minimumBoundingBoxSize, minimumBoundingBoxSize);
   }

   default void setPlaneNoiseVariance(double planeNoiseVariance)
   {
      set(PlanarRegionMappingParameters.planeNoiseVariance, planeNoiseVariance);
   }

   default void setOdometryNoiseVariance(double odometryNoiseVariance)
   {
      set(PlanarRegionMappingParameters.odometryNoiseVariance, odometryNoiseVariance);
   }

   default void setTerminationRatio(double terminationRatio)
   {
      set(PlanarRegionMappingParameters.terminationRatio, terminationRatio);
   }

   default void setICPMaxIterations(int icpMaxIterations)
   {
      set(PlanarRegionMappingParameters.icpMaxIterations, icpMaxIterations);
   }

   default void setMaxRegistrationError(double maxRegistrationError)
   {
      set(PlanarRegionMappingParameters.maxRegistrationError, maxRegistrationError);
   }

   default void setBestMatchAngularThreshold(double bestMatchAngularThreshold)
   {
      set(PlanarRegionMappingParameters.bestMatchAngularThreshold, bestMatchAngularThreshold);
   }

   default void setBestMatchDistanceThreshold(double bestMatchDistanceThreshold)
   {
      set(PlanarRegionMappingParameters.bestMatchDistanceThreshold, bestMatchDistanceThreshold);
   }

   default void setBestMinimumOverlapThreshold(double bestMinimumOverlapThreshold)
   {
      set(PlanarRegionMappingParameters.bestMinimumOverlapThreshold, bestMinimumOverlapThreshold);
   }

   default void setICPMinMatches(int icpMinMatches)
   {
      set(PlanarRegionMappingParameters.icpMinMatches, icpMinMatches);
   }
}
