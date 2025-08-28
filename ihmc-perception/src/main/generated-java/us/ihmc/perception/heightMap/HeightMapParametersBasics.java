package us.ihmc.perception.heightMap;

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

   default void setEnableChunkedMap(boolean enableChunkedMap)
   {
      set(HeightMapParameters.enableChunkedMap, enableChunkedMap);
   }

   default void setLogHeightMap(boolean logHeightMap)
   {
      set(HeightMapParameters.logHeightMap, logHeightMap);
   }

   default void setSearchWindowHeight(int searchWindowHeight)
   {
      set(HeightMapParameters.searchWindowHeight, searchWindowHeight);
   }

   default void setSearchWindowWidth(int searchWindowWidth)
   {
      set(HeightMapParameters.searchWindowWidth, searchWindowWidth);
   }

   default void setSearchSkipSize(int searchSkipSize)
   {
      set(HeightMapParameters.searchSkipSize, searchSkipSize);
   }

   default void setMinHeightRegistration(double minHeightRegistration)
   {
      set(HeightMapParameters.minHeightRegistration, minHeightRegistration);
   }

   default void setMaxHeightRegistration(double maxHeightRegistration)
   {
      set(HeightMapParameters.maxHeightRegistration, maxHeightRegistration);
   }

   default void setMinHeightDifference(double minHeightDifference)
   {
      set(HeightMapParameters.minHeightDifference, minHeightDifference);
   }

   default void setMaxHeightDifference(double maxHeightDifference)
   {
      set(HeightMapParameters.maxHeightDifference, maxHeightDifference);
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

   default void setTerrainWidthInMeters(double terrainWidthInMeters)
   {
      set(HeightMapParameters.terrainWidthInMeters, terrainWidthInMeters);
   }

   default void setGlobalWidthInMeters(double globalWidthInMeters)
   {
      set(HeightMapParameters.globalWidthInMeters, globalWidthInMeters);
   }

   default void setSteppingContactThreshold(int steppingContactThreshold)
   {
      set(HeightMapParameters.steppingContactThreshold, steppingContactThreshold);
   }

   default void setContactWindowSize(int contactWindowSize)
   {
      set(HeightMapParameters.contactWindowSize, contactWindowSize);
   }

   default void setSteppingCosineThreshold(double steppingCosineThreshold)
   {
      set(HeightMapParameters.steppingCosineThreshold, steppingCosineThreshold);
   }
}
