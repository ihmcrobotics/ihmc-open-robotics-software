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

   default double getSimilarityThresholdBetweenNormals()
   {
      return get(similarityThresholdBetweenNormals);
   }

   default double getOrthogonalDistanceThreshold()
   {
      return get(orthogonalDistanceThreshold);
   }

   default double getMaxInterRegionDistance()
   {
      return get(maxInterRegionDistance);
   }

   default double getMinimumOverlapThreshold()
   {
      return get(minimumOverlapThreshold);
   }

   default double getMinimumPlanarRegionArea()
   {
      return get(minimumPlanarRegionArea);
   }

   default double getMinimumBoundingBoxSize()
   {
      return get(minimumBoundingBoxSize);
   }

   default double getPlaneNoiseVariance()
   {
      return get(planeNoiseVariance);
   }

   default double getOdometryNoiseVariance()
   {
      return get(odometryNoiseVariance);
   }

   default double getStateEstimatorNoiseVariance()
   {
      return get(stateEstimatorNoiseVariance);
   }

   default double getBestMatchAngularThreshold()
   {
      return get(bestMatchAngularThreshold);
   }

   default double getBestMatchDistanceThreshold()
   {
      return get(bestMatchDistanceThreshold);
   }

   default double getBestMinimumOverlapThreshold()
   {
      return get(bestMinimumOverlapThreshold);
   }

   default double getKeyframeDistanceThreshold()
   {
      return get(keyframeDistanceThreshold);
   }

   default double getKeyframeAngularThreshold()
   {
      return get(keyframeAngularThreshold);
   }

   default int getICPMaxIterations()
   {
      return get(icpMaxIterations);
   }

   default int getICPMinMatches()
   {
      return get(icpMinMatches);
   }

   default double getICPTerminationRatio()
   {
      return get(icpTerminationRatio);
   }

   default double getICPErrorCutoff()
   {
      return get(icpErrorCutoff);
   }

   default int getMinimumNumberOfTimesMatched()
   {
      return get(minimumNumberOfTimesMatched);
   }
}
