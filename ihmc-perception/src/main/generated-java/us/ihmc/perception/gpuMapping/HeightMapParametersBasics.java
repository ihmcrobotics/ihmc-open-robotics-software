package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapParametersBasics extends HeightMapParametersReadOnly, StoredPropertySetBasics
{
   default void setDriftOffsetFilter(boolean driftOffsetFilter)
   {
      set(HeightMapParameters.driftOffsetFilter, driftOffsetFilter);
   }

   default void setFlyingPointsFilter(boolean flyingPointsFilter)
   {
      set(HeightMapParameters.flyingPointsFilter, flyingPointsFilter);
   }

   default void setMinDepthToAccept(double minDepthToAccept)
   {
      set(HeightMapParameters.minDepthToAccept, minDepthToAccept);
   }

   default void setMinHeightRegistration(double minHeightRegistration)
   {
      set(HeightMapParameters.minHeightRegistration, minHeightRegistration);
   }

   default void setMaxHeightRegistration(double maxHeightRegistration)
   {
      set(HeightMapParameters.maxHeightRegistration, maxHeightRegistration);
   }

   default void setKalmanFilterPredictionNoise(double kalmanFilterPredictionNoise)
   {
      set(HeightMapParameters.kalmanFilterPredictionNoise, kalmanFilterPredictionNoise);
   }

   default void setAdditionalTranslationalVarianceAdded(double additionalTranslationalVarianceAdded)
   {
      set(HeightMapParameters.additionalTranslationalVarianceAdded, additionalTranslationalVarianceAdded);
   }

   default void setVariancePerMeter(double variancePerMeter)
   {
      set(HeightMapParameters.variancePerMeter, variancePerMeter);
   }

   default void setVariancePerTranslationSpeed(double variancePerTranslationSpeed)
   {
      set(HeightMapParameters.variancePerTranslationSpeed, variancePerTranslationSpeed);
   }

   default void setVariancePerRotationSpeed(double variancePerRotationSpeed)
   {
      set(HeightMapParameters.variancePerRotationSpeed, variancePerRotationSpeed);
   }

   default void setMinClampHeight(double minClampHeight)
   {
      set(HeightMapParameters.minClampHeight, minClampHeight);
   }

   default void setMaxClampHeight(double maxClampHeight)
   {
      set(HeightMapParameters.maxClampHeight, maxClampHeight);
   }

   default void setCellSize(double cellSize)
   {
      set(HeightMapParameters.cellSize, cellSize);
   }

   default void setLocalWidthInMeters(double localWidthInMeters)
   {
      set(HeightMapParameters.localWidthInMeters, localWidthInMeters);
   }

   default void setGlobalWidthInMeters(double globalWidthInMeters)
   {
      set(HeightMapParameters.globalWidthInMeters, globalWidthInMeters);
   }

   default void setIcpMaxIterations(int icpMaxIterations)
   {
      set(HeightMapParameters.icpMaxIterations, icpMaxIterations);
   }

   default void setIcpConvergence(double icpConvergence)
   {
      set(HeightMapParameters.icpConvergence, icpConvergence);
   }

   default void setIcpMaxDistance(double icpMaxDistance)
   {
      set(HeightMapParameters.icpMaxDistance, icpMaxDistance);
   }
}
