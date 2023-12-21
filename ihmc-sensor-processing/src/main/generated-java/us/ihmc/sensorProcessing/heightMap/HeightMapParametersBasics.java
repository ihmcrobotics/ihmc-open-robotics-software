package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapParametersBasics extends HeightMapParametersReadOnly, StoredPropertySetBasics
{
   default void setResetHeightMap(boolean resetHeightMap)
   {
      set(HeightMapParameters.resetHeightMap, resetHeightMap);
   }

   default void setSearchWindowHeight(int searchWindowHeight)
   {
      set(HeightMapParameters.searchWindowHeight, searchWindowHeight);
   }

   default void setSearchWindowWidth(int searchWindowWidth)
   {
      set(HeightMapParameters.searchWindowWidth, searchWindowWidth);
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

   default void setHeightFilterAlpha(double heightFilterAlpha)
   {
      set(HeightMapParameters.heightFilterAlpha, heightFilterAlpha);
   }

   default void setSpatialAlpha(double spatialAlpha)
   {
      set(HeightMapParameters.spatialAlpha, spatialAlpha);
   }

   default void setHeightOffset(double heightOffset)
   {
      set(HeightMapParameters.heightOffset, heightOffset);
   }

   default void setMinClampHeight(double minClampHeight)
   {
      set(HeightMapParameters.minClampHeight, minClampHeight);
   }

   default void setMaxClampHeight(double maxClampHeight)
   {
      set(HeightMapParameters.maxClampHeight, maxClampHeight);
   }

   default void setLocalWidthInMeters(double localWidthInMeters)
   {
      set(HeightMapParameters.localWidthInMeters, localWidthInMeters);
   }

   default void setLocalCellSizeInMeters(double localCellSizeInMeters)
   {
      set(HeightMapParameters.localCellSizeInMeters, localCellSizeInMeters);
   }

   default void setGlobalWidthInMeters(double globalWidthInMeters)
   {
      set(HeightMapParameters.globalWidthInMeters, globalWidthInMeters);
   }

   default void setGlobalCellSizeInMeters(double globalCellSizeInMeters)
   {
      set(HeightMapParameters.globalCellSizeInMeters, globalCellSizeInMeters);
   }

   default void setRobotCollisionCylinderRadius(double robotCollisionCylinderRadius)
   {
      set(HeightMapParameters.robotCollisionCylinderRadius, robotCollisionCylinderRadius);
   }

   default void setInternalGlobalWidthInMeters(double internalGlobalWidthInMeters)
   {
      set(HeightMapParameters.internalGlobalWidthInMeters, internalGlobalWidthInMeters);
   }

   default void setInternalGlobalCellSizeInMeters(double internalGlobalCellSizeInMeters)
   {
      set(HeightMapParameters.internalGlobalCellSizeInMeters, internalGlobalCellSizeInMeters);
   }

   default void setHeightScaleFactor(double heightScaleFactor)
   {
      set(HeightMapParameters.heightScaleFactor, heightScaleFactor);
   }

   default void setCropWindowSize(int cropWindowSize)
   {
      set(HeightMapParameters.cropWindowSize, cropWindowSize);
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

   default void setSearchSkipSize(int searchSkipSize)
   {
      set(HeightMapParameters.searchSkipSize, searchSkipSize);
   }

   default void setFastSearchSize(int fastSearchSize)
   {
      set(HeightMapParameters.fastSearchSize, fastSearchSize);
   }

   default void setVerticalSearchSize(int verticalSearchSize)
   {
      set(HeightMapParameters.verticalSearchSize, verticalSearchSize);
   }

   default void setVerticalSearchResolution(double verticalSearchResolution)
   {
      set(HeightMapParameters.verticalSearchResolution, verticalSearchResolution);
   }

   /**
    * Resolution of the height map grid
    */
   default void setGridResolutionXY(double gridResolutionXY)
   {
      set(HeightMapParameters.gridResolutionXY, gridResolutionXY);
   }

   /**
    * Length of the side of the square height map grid
    */
   default void setGridSizeXY(double gridSizeXY)
   {
      set(HeightMapParameters.gridSizeXY, gridSizeXY);
   }

   /**
    * Max z relative to robot mid foot z. Points above this threshold are ignored.
    */
   default void setMaxZ(double maxZ)
   {
      set(HeightMapParameters.maxZ, maxZ);
   }

   /**
    * When calibrated on flat ground, this is the average standard deviation observed
    * for a grid cell.
    */
   default void setNominalStandardDeviation(double nominalStandardDeviation)
   {
      set(HeightMapParameters.nominalStandardDeviation, nominalStandardDeviation);
   }

   default void setMaxPointsPerCell(int maxPointsPerCell)
   {
      set(HeightMapParameters.maxPointsPerCell, maxPointsPerCell);
   }

   /**
    * If a grid cell is at height h, points below (h - s * m) are ignored, and points
    * above (h + s * m) will cause the cell to throw out old data and reset. where s
    * is getNominalStandardDeviation() and m is this value.
    */
   default void setMahalanobisScale(double mahalanobisScale)
   {
      set(HeightMapParameters.mahalanobisScale, mahalanobisScale);
   }

   /**
    * This is the variance added to all past measurements when a cell is translated
    */
   default void setVarianceAddedWhenTranslating(double varianceAddedWhenTranslating)
   {
      set(HeightMapParameters.varianceAddedWhenTranslating, varianceAddedWhenTranslating);
   }

   /**
    * This is the measurement variance when the robot is standing
    */
   default void setSensorVarianceWhenStanding(double sensorVarianceWhenStanding)
   {
      set(HeightMapParameters.sensorVarianceWhenStanding, sensorVarianceWhenStanding);
   }

   /**
    * This is the measurement variance when the robot is moving
    */
   default void setSensorVarianceWhenMoving(double sensorVarianceWhenMoving)
   {
      set(HeightMapParameters.sensorVarianceWhenMoving, sensorVarianceWhenMoving);
   }

   default void setEstimateHeightWithKalmanFilter(boolean estimateHeightWithKalmanFilter)
   {
      set(HeightMapParameters.estimateHeightWithKalmanFilter, estimateHeightWithKalmanFilter);
   }

   default void setDenoiserEnabled(boolean denoiserEnabled)
   {
      set(HeightMapParameters.denoiserEnabled, denoiserEnabled);
   }

   default void setStatisticsLoggingEnabled(boolean statisticsLoggingEnabled)
   {
      set(HeightMapParameters.statisticsLoggingEnabled, statisticsLoggingEnabled);
   }
}
