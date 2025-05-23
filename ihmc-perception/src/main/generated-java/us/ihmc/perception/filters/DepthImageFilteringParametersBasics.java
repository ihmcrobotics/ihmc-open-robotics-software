package us.ihmc.perception.filters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface DepthImageFilteringParametersBasics extends DepthImageFilteringParametersReadOnly, StoredPropertySetBasics
{
   default void setWindowSizeInPixels(int windowSizeInPixels)
   {
      set(DepthImageFilteringParameters.windowSizeInPixels, windowSizeInPixels);
   }

   default void setAngleThresholdInRadians(double angleThresholdInRadians)
   {
      set(DepthImageFilteringParameters.angleThresholdInRadians, angleThresholdInRadians);
   }

   default void setNormalAngleThreshold(double normalAngleThreshold)
   {
      set(DepthImageFilteringParameters.normalAngleThreshold, normalAngleThreshold);
   }

   default void setRANSACIterations(int ransacIterations)
   {
      set(DepthImageFilteringParameters.ransacIterations, ransacIterations);
   }

   default void setMinimumDepthValuesRequiredInWindow(int minimumDepthValuesRequiredInWindow)
   {
      set(DepthImageFilteringParameters.minimumDepthValuesRequiredInWindow, minimumDepthValuesRequiredInWindow);
   }
}
