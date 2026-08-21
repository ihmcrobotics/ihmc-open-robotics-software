package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.gpuMapping.HeightMapParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getDriftOffsetFilter()
   {
      return get(driftOffsetFilter);
   }

   default boolean getFlyingPointsFilter()
   {
      return get(flyingPointsFilter);
   }

   default boolean getLogHeightMap()
   {
      return get(logHeightMap);
   }

   default double getMinDepthToAccept()
   {
      return get(minDepthToAccept);
   }

   default double getMinHeightRegistration()
   {
      return get(minHeightRegistration);
   }

   default double getMaxHeightRegistration()
   {
      return get(maxHeightRegistration);
   }

   default double getKalmanFilterPredictionNoise()
   {
      return get(kalmanFilterPredictionNoise);
   }

   default double getAdditionalTranslationalVarianceAdded()
   {
      return get(additionalTranslationalVarianceAdded);
   }

   default double getVariancePerMeter()
   {
      return get(variancePerMeter);
   }

   default double getVariancePerTranslationSpeed()
   {
      return get(variancePerTranslationSpeed);
   }

   default double getVariancePerRotationSpeed()
   {
      return get(variancePerRotationSpeed);
   }

   default double getMinClampHeight()
   {
      return get(minClampHeight);
   }

   default double getMaxClampHeight()
   {
      return get(maxClampHeight);
   }

   default double getCellSize()
   {
      return get(cellSize);
   }

   default double getLocalWidthInMeters()
   {
      return get(localWidthInMeters);
   }

   default double getGlobalWidthInMeters()
   {
      return get(globalWidthInMeters);
   }

   default boolean getIcpRotationEnabled()
   {
      return get(icpRotationEnabled);
   }

   default int getIcpMaxIterations()
   {
      return get(icpMaxIterations);
   }

   default int getIcpMinCorrespondenceCount()
   {
      return get(icpMinCorrespondenceCount);
   }

   default double getIcpMaxHorizontalDrift()
   {
      return get(icpMaxHorizontalDrift);
   }

   default double getIcpOutlierDistanceThreshold()
   {
      return get(icpOutlierDistanceThreshold);
   }

   default double getIcpVariancePerMeterOfCorrection()
   {
      return get(icpVariancePerMeterOfCorrection);
   }

   default double getIcpConvergenceZMeters()
   {
      return get(icpConvergenceZMeters);
   }

   default double getIcpConvergenceYawDegrees()
   {
      return get(icpConvergenceYawDegrees);
   }
}
