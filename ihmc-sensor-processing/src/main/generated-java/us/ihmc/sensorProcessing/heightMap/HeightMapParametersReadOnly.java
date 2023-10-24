package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.sensorProcessing.heightMap.HeightMapParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * Resolution of the height map grid
    */
   default double getGridResolutionXY()
   {
      return get(gridResolutionXY);
   }

   /**
    * Length of the side of the square height map grid
    */
   default double getGridSizeXY()
   {
      return get(gridSizeXY);
   }

   /**
    * Max z relative to robot mid foot z. Points above this threshold are ignored.
    */
   default double getMaxZ()
   {
      return get(maxZ);
   }

   /**
    * When calibrated on flat ground, this is the average standard deviation observed
    * for a grid cell.
    */
   default double getNominalStandardDeviation()
   {
      return get(nominalStandardDeviation);
   }

   default int getMaxPointsPerCell()
   {
      return get(maxPointsPerCell);
   }

   /**
    * If a grid cell is at height h, points below (h - s * m) are ignored, and points
    * above (h + s * m) will cause the cell to throw out old data and reset. where s
    * is getNominalStandardDeviation() and m is this value.
    */
   default double getMahalanobisScale()
   {
      return get(mahalanobisScale);
   }

   /**
    * This is the variance added to all past measurements when a cell is translated
    */
   default double getVarianceAddedWhenTranslating()
   {
      return get(varianceAddedWhenTranslating);
   }

   /**
    * This is the measurement variance when the robot is standing
    */
   default double getSensorVarianceWhenStanding()
   {
      return get(sensorVarianceWhenStanding);
   }

   /**
    * This is the measurement variance when the robot is moving
    */
   default double getSensorVarianceWhenMoving()
   {
      return get(sensorVarianceWhenMoving);
   }

   default boolean getEstimateHeightWithKalmanFilter()
   {
      return get(estimateHeightWithKalmanFilter);
   }

   default boolean getResetHeightMap()
   {
      return get(resetHeightMap);
   }
}
