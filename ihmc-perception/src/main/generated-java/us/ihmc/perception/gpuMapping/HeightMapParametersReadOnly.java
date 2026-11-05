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

   default boolean getICPFilter()
   {
      return get(icpFilter);
   }

   default boolean getICPRotationEnabled()
   {
      return get(icpRotationEnabled);
   }

   default int getICPMaxIterations()
   {
      return get(icpMaxIterations);
   }

   default int getICPMinCorrespondenceCount()
   {
      return get(icpMinCorrespondenceCount);
   }

   default double getICPMaxHorizontalDrift()
   {
      return get(icpMaxHorizontalDrift);
   }

   default double getICPOutlierDistanceThreshold()
   {
      return get(icpOutlierDistanceThreshold);
   }

   default double getICPMinDistance()
   {
      return get(icpMinDistance);
   }

   default double getICPMaxDistance()
   {
      return get(icpMaxDistance);
   }

   default double getICPNeighborHeightThreshold()
   {
      return get(icpNeighborHeightThreshold);
   }

   default int getICPMinConsistentNeighbors()
   {
      return get(icpMinConsistentNeighbors);
   }

   default int getICPMinPointsPerCell()
   {
      return get(icpMinPointsPerCell);
   }

   default double getICPRobustKernelWidth()
   {
      return get(icpRobustKernelWidth);
   }

   default double getICPMinZCorrection()
   {
      return get(icpMinZCorrection);
   }

   default double getICPMinXYCorrection()
   {
      return get(icpMinXYCorrection);
   }

   default double getICPMinYawCorrectionDegrees()
   {
      return get(icpMinYawCorrectionDegrees);
   }

   default double getICPVariancePerMeterOfCorrection()
   {
      return get(icpVariancePerMeterOfCorrection);
   }

   default double getICPConvergenceZMeters()
   {
      return get(icpConvergenceZMeters);
   }

   default double getICPConvergenceYawDegrees()
   {
      return get(icpConvergenceYawDegrees);
   }

   default boolean getFillUnseenCells()
   {
      return get(fillUnseenCells);
   }

   default double getFillBlendStartDistance()
   {
      return get(fillBlendStartDistance);
   }
}
