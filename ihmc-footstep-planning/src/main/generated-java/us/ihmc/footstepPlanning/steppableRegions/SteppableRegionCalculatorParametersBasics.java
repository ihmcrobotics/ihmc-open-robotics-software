package us.ihmc.footstepPlanning.steppableRegions;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface SteppableRegionCalculatorParametersBasics extends SteppableRegionCalculatorParametersReadOnly, StoredPropertySetBasics
{
   default void setNormalSearchRadius(double normalSearchRadius)
   {
      set(SteppableRegionCalculatorParameters.normalSearchRadius, normalSearchRadius);
   }

   default void setMinSupportAreaFraction(double minSupportAreaFraction)
   {
      set(SteppableRegionCalculatorParameters.minSupportAreaFraction, minSupportAreaFraction);
   }

   default void setMinSnapHeightThreshold(double minSnapHeightThreshold)
   {
      set(SteppableRegionCalculatorParameters.minSnapHeightThreshold, minSnapHeightThreshold);
   }

   default void setSnapHeightThresholdAtSearchEdge(double snapHeightThresholdAtSearchEdge)
   {
      set(SteppableRegionCalculatorParameters.snapHeightThresholdAtSearchEdge, snapHeightThresholdAtSearchEdge);
   }

   default void setSteppingCosineThreshold(double steppingCosineThreshold)
   {
      set(SteppableRegionCalculatorParameters.steppingCosineThreshold, steppingCosineThreshold);
   }

   default void setSquaredErrorThreshold(double squaredErrorThreshold)
   {
      set(SteppableRegionCalculatorParameters.squaredErrorThreshold, squaredErrorThreshold);
   }
}
