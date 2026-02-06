package us.ihmc.perception.gpuMapping;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface TerrainMapParametersBasics extends TerrainMapParametersReadOnly, StoredPropertySetBasics
{
   default void setNormalSearchRadius(double normalSearchRadius)
   {
      set(TerrainMapParameters.normalSearchRadius, normalSearchRadius);
   }

   default void setCliffSearchRadius(double cliffSearchRadius)
   {
      set(TerrainMapParameters.cliffSearchRadius, cliffSearchRadius);
   }

   default void setCliffHeightThreshold(double cliffHeightThreshold)
   {
      set(TerrainMapParameters.cliffHeightThreshold, cliffHeightThreshold);
   }

   default void setCliffHeightTolerance(double cliffHeightTolerance)
   {
      set(TerrainMapParameters.cliffHeightTolerance, cliffHeightTolerance);
   }

   default void setMinSupportAreaFraction(double minSupportAreaFraction)
   {
      set(TerrainMapParameters.minSupportAreaFraction, minSupportAreaFraction);
   }

   default void setMinSnapHeightThreshold(double minSnapHeightThreshold)
   {
      set(TerrainMapParameters.minSnapHeightThreshold, minSnapHeightThreshold);
   }

   default void setSnapHeightThresholdAtSearchEdge(double snapHeightThresholdAtSearchEdge)
   {
      set(TerrainMapParameters.snapHeightThresholdAtSearchEdge, snapHeightThresholdAtSearchEdge);
   }

   default void setSteppingCosineThreshold(double steppingCosineThreshold)
   {
      set(TerrainMapParameters.steppingCosineThreshold, steppingCosineThreshold);
   }

   default void setSquaredErrorThreshold(double squaredErrorThreshold)
   {
      set(TerrainMapParameters.squaredErrorThreshold, squaredErrorThreshold);
   }
}
