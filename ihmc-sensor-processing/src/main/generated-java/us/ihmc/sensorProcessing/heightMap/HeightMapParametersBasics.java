package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapParametersBasics extends HeightMapParametersReadOnly, StoredPropertySetBasics
{
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
}
