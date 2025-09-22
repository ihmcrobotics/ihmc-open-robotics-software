package us.ihmc.perception.heightMap;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.perception.heightMap.HeightMapParameters.*;

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

   default boolean getEnableChunkedMap()
   {
      return get(enableChunkedMap);
   }

   default boolean getLogHeightMap()
   {
      return get(logHeightMap);
   }

   default int getSearchWindowHeight()
   {
      return get(searchWindowHeight);
   }

   default int getSearchWindowWidth()
   {
      return get(searchWindowWidth);
   }

   default int getSearchSkipSize()
   {
      return get(searchSkipSize);
   }

   default double getMinHeightRegistration()
   {
      return get(minHeightRegistration);
   }

   default double getMaxHeightRegistration()
   {
      return get(maxHeightRegistration);
   }

   default double getMinHeightDifference()
   {
      return get(minHeightDifference);
   }

   default double getMaxHeightDifference()
   {
      return get(maxHeightDifference);
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

   default double getTerrainWidthInMeters()
   {
      return get(terrainWidthInMeters);
   }

   default double getGlobalWidthInMeters()
   {
      return get(globalWidthInMeters);
   }
}
